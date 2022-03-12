package DFRobot_DC_Motor

import (
	"time"

	i2c "github.com/d2r2/go-i2c"
	logger "github.com/d2r2/go-logger"
)

// Register number
type Register byte

const (
	_REG_SLAVE_ADDR               Register = 0x00
	_REG_PID                               = 0x01
	_REG_PVD                               = 0x02
	_REG_CTRL_MODE                         = 0x03
	_REG_ENCODER1_EN                       = 0x04
	_REG_ENCODER1_SPPED                    = 0x05
	_REG_ENCODER1_REDUCTION_RATIO          = 0x07
	_REG_ENCODER2_EN                       = 0x09
	_REG_ENCODER2_SPEED                    = 0x0a
	_REG_ENCODER2_REDUCTION_RATIO          = 0x0c
	_REG_MOTOR_PWM                         = 0x0e
	_REG_MOTOR1_ORIENTATION                = 0x0f
	_REG_MOTOR1_SPEED                      = 0x10
	_REG_MOTOR2_ORIENTATION                = 0x12
	_REG_MOTOR2_SPEED                      = 0x13

	_REG_DEF_PID = 0xdf
	_REG_DEF_VID = 0x10
)

// Enum motor ID
type MotorId byte

const (
	M1 MotorId = 0x01
	M2 MotorId = 0x02
)

// Board status
type Status byte

const (
	STA_OK                      Status = 0x00
	STA_ERR                     Status = 0x01
	STA_ERR_DEVICE_NOT_DETECTED Status = 0x02
	STA_ERR_SOFT_VERSION        Status = 0x03
	STA_ERR_PARAMETER           Status = 0x04
)

// Orientation
type Direction byte

const (
	CW   Direction = 0x01 // clockwise
	CCW  Direction = 0x02 // countclockwise
	STOP Direction = 0x05 // stop
)

var lg = logger.NewPackageLogger("DC",
	logger.DebugLevel,
	// logger.InfoLevel,
)

type DFMotor struct {
	i2cbus *i2c.I2C
	Addr   byte
}

func checkBoard(i2cbus *i2c.I2C) bool {
	pid, err := i2cbus.ReadRegU8(_REG_PID)
	if err != nil {
		return false
	}
	vid, err := i2cbus.ReadRegU8(_REG_PVD)
	if err != nil {
		return false
	}
	lg.Debugf("pid:%d vid:%d\n", pid, vid)
	if pid != _REG_DEF_PID || vid != _REG_DEF_VID {
		return false
	}
	return true
}

func New(addr byte) *DFMotor {
	var drv DFMotor
	var err error
	drv.Addr = addr
	var succesfull bool = false
	drv.i2cbus, err = i2c.NewI2C(drv.Addr, 1)

	if err != nil {
		lg.Error(err)
		return nil
	}
	defer func() {
		if !succesfull {
			drv.i2cbus.Close()
		}
	}()

	if !checkBoard(drv.i2cbus) {
		lg.Fatal("device not detected")
		return nil
	}

	// set DC motor mode
	drv.i2cbus.WriteRegU8(_REG_CTRL_MODE, 0)
	drv.MotorStop(M1)
	drv.MotorStop(M2)
	drv.SetEncoderDisable(M1)
	drv.SetEncoderDisable(M2)
	succesfull = true
	return &drv
}

func (drv *DFMotor) Close() {
	if drv != nil && drv.i2cbus != nil {
		drv.MotorStop(M1)
		drv.MotorStop(M2)
		drv.i2cbus.Close()
		drv.i2cbus = nil
	}
}

func (drv *DFMotor) SetEncoderEnable(id MotorId) bool {
	err := drv.i2cbus.WriteRegU8(byte(_REG_ENCODER1_EN+5*(id-1)), 0x01)
	if err != nil {
		lg.Error(err)
		return false
	}
	lg.Debugf("M%d encoder enable\n", int(id))
	return true
}

func (drv *DFMotor) SetEncoderDisable(id MotorId) bool {
	err := drv.i2cbus.WriteRegU8(byte(_REG_ENCODER1_EN+5*(id-1)), 0x0)
	if err != nil {
		lg.Error(err)
		return false
	}
	lg.Debugf("M%d encoder disable\n", int(id))
	return true
}

func (drv *DFMotor) SetEncoderReductionRatio(id MotorId, reductionRatio uint16) bool {
	if reductionRatio < 1 || reductionRatio > 2000 {
		lg.Error("reductionRatio out of range: 1-2000")
		return false
	}
	err := drv.i2cbus.WriteRegU16BE(byte(_REG_ENCODER1_REDUCTION_RATIO+5*(id-1)), reductionRatio)
	lg.Debugf("M%d set reduction ratio:%d\n", int(id), int(reductionRatio))
	if err != nil {
		lg.Error(err)
		return false
	}
	return true
}

func (drv *DFMotor) GetEncoderSpeed(id MotorId) int32 {
	s, err := drv.i2cbus.ReadRegU16BE(byte(_REG_ENCODER1_SPPED + 5*(id-1)))
	if err != nil {
		lg.Error(err)
		return 0
	}
	if s&0x8000 > 0 {
		return int32(-(0x10000 - uint32(s)))
	}
	return int32(s)
}

func (drv *DFMotor) SetMoterPwmFrequency(frequency int) bool {
	if frequency < 100 || frequency > 12750 {
		lg.Error("frequency out of range: 100-12750")
		return false
	}
	err := drv.i2cbus.WriteRegU8(_REG_CTRL_MODE, byte(frequency/50))
	if err != nil {
		lg.Error(err)
		return false
	}
	lg.Debugf("set motors PWM:%d\n", frequency)
	time.Sleep(100 * time.Millisecond)
	return true
}

// Motor movement
// id: MotorId          Motor Id M1 or M2
// direction: Direction Motor orientation, CW (clockwise) or CCW (counterclockwise)
// speed: float         Motor pwm duty cycle, in range 0 to 100, otherwise no effective
func (drv *DFMotor) MotorMovement(id MotorId, direction Direction, speed float32) bool {
	if direction != CW && direction != CCW {
		return false
	}
	if speed < 0.0 || speed > 100.0 {
		return false
	}
	reg := byte(_REG_MOTOR1_ORIENTATION + (id-1)*3)
	err := drv.i2cbus.WriteRegU8(reg, byte(direction))
	if err != nil {
		lg.Error(err)
		return false
	}
	var sp uint16 = uint16(speed)<<8 + uint16(speed*10.0)%10

	err = drv.i2cbus.WriteRegU16BE(reg+1, sp)
	if err != nil {
		lg.Error(err)
		return false
	}

	var dirStr string
	if direction == CW {
		dirStr = "CW"
	} else {
		dirStr = "CCW"
	}

	lg.Debugf("M%d movement Dir:%s, Speed:%f\n", int(id), dirStr, speed)
	return true
}

// Motor stop
// id: MotorId          Motor Id M1 or M2
func (drv *DFMotor) MotorStop(id MotorId) bool {
	err := drv.i2cbus.WriteRegU8(byte(_REG_MOTOR1_ORIENTATION+3*(id-1)), byte(STOP))
	if err != nil {
		lg.Error(err)
		return false
	}
	lg.Debugf("M%d stop\n", int(id))
	return true
}

//  Set board controler address, reboot module to make it effective
//  param address: byte    Address to set, range in 1 to 127
func (drv *DFMotor) SetAddr(addr byte) bool {
	if addr < 1 || addr > 127 {
		lg.Errorf("addres out of range (1..127)\n")
		return false
	}
	err := drv.i2cbus.WriteRegU8(byte(_REG_SLAVE_ADDR), addr)
	if err != nil {
		lg.Error(err)
		return false
	}
	lg.Debugf("new addfes:%02X\n", addr)
	return true
}

func Detecte() []byte {
	var addrList []byte
	for addr := uint8(1); addr < 128; addr++ {
		i2cbus, err := i2c.NewI2C(addr, 1)
		if err == nil {
			if checkBoard(i2cbus) {
				addrList = append(addrList, addr)
			}
			i2cbus.Close()
		}
	}
	return addrList
}
