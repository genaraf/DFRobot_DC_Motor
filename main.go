package main

import (
	"fmt"
	motor "motor/DFRobot_DC_Motor"
	"time"
)

func main() {
	fmt.Println("Start DFRobot_DC_Motor")
	devlst := motor.Detecte()
	fmt.Printf("Board addreses:%v\n", devlst)
	m := motor.New(0x10)
	fmt.Printf("Motor:%v\n", m)
	m.SetMoterPwmFrequency(3000)
	m.SetEncoderEnable(motor.M1)
	m.SetEncoderEnable(motor.M2)
	m.SetEncoderReductionRatio(motor.M1, 49)
	m.SetEncoderReductionRatio(motor.M2, 49)
	for i := float32(10.0); i <= 100.0; i += 10.0 {
		m.MotorMovement(motor.M1, motor.CW, i)
		m.MotorMovement(motor.M2, motor.CCW, i)
		time.Sleep(2 * time.Second)
		s := m.GetEncoderSpeed(motor.M1)
		fmt.Printf("M1 speed: %.2f, encoder speed:%d\n", i, s)
		s = m.GetEncoderSpeed(motor.M2)
		fmt.Printf("M2 speed: %.2f, encoder speed:%d\n", i, s)
	}
	m.MotorStop(motor.M1)
	m.Close()
}
