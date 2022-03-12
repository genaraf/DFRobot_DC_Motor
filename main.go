package main

import (
	"fmt"
	motor "motor/DFRobot_DC_Motor"
)

func main() {
	fmt.Println("Hello UbuntuPIT")
	m := motor.New()
	fmt.Printf("Motor:%v\n", m)
	m.Close()
}
