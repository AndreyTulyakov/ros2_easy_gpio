#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from easy_gpio_interfaces.srv import ReadBit, WriteBit, WritePWM

from easy_gpio.lgpio_patch import lgpio


class GPIOServiceNode(Node):

    def __init__(self, name: str = 'gpio_node', **kwargs):
        super().__init__(name, **kwargs)
        self.input_pins = {}
        self.output_pins = {}

        self.srv_read_bit = self.create_service(ReadBit, f'gpio/read_bit', self.readBit)
        self.srv_write_bit = self.create_service(WriteBit, f'gpio/write_bit', self.writeBit)
        self.srv_write_pwm = self.create_service(WritePWM, f'gpio/write_pwm', self.writePWM)

        self.chip_handler = lgpio.gpiochip_open(0)
        if self.chip_handler < 0:
            error = f'Can not open GPIO device! Error: {self.chip_handler}'
            self.get_logger().error(error)
            raise RuntimeError(error)

        chip_info = lgpio.gpio_get_chip_info(self.chip_handler)
        self.get_logger().info(f'Chip info: lines={chip_info[1]} name={chip_info[2]} label={chip_info[3]}')

    def readBit(self, request: ReadBit, response):
        pin = request.pin

        try:
            if pin in self.output_pins:
                raise RuntimeError(f'Pin "{pin}" already used as output pin!')
              
            if pin not in self.input_pins:
                if request.pull == "UP":
                    pull_mode = lgpio.SET_PULL_UP
                elif request.pull == "DOWN":
                    pull_mode = lgpio.SET_PULL_DOWN
                else:
                    pull_mode = lgpio.SET_PULL_NONE

                self.input_pins[pin] = lgpio.gpio_claim_input(self.chip_handler, pin, pull_mode)

            value = lgpio.gpio_read(self.chip_handler, pin)

            if value not in (0, 1):
                raise RuntimeError(f'GPIO "{pin}" read error: {value}')

            response.value = bool(value)
        except Exception as e:
            msg = e.args[0]
            self.get_logger().error(msg)
            response.error = msg

        return response

    def writeBit(self, request, response):
        pin = request.pin

        try:
            self._prepare_output_pin(pin, request.mode)
            result = lgpio.gpio_write(self.chip_handler, pin, int(bool(request.value)))
            if result < 0:
                raise RuntimeError(f'Pin "{pin}" write error: {result}')
            
        except Exception as e:
            msg = e.args[0]
            self.get_logger().error(msg)
            response.error = msg
        return response

    def writePWM(self, request, response):
        pin = request.pin

        try:
            self._prepare_output_pin(pin, request.mode)
            result = lgpio.tx_pwm(self.chip_handler, pin, request.frequency, request.duty)
            if result < 0:
                raise RuntimeError(f'Pin "{pin}" set PWM error: {result}')
            
        except Exception as e:
            msg = e.args[0]
            self.get_logger().error(msg)
            response.error = msg
        return response

    def _prepare_output_pin(self, pin: int, mode_line: str):
        if pin in self.input_pins:
            raise RuntimeError(f'Pin "{pin}" already used as input pin!')
    
        flags = 0;
        if mode_line:
            modes = [m.strip() for m in mode_line.split('|')]
            if 'OPEN_SOURCE' in modes:
                flags = flags | lgpio.SET_OPEN_SOURCE
            if 'OPEN_DRAIN' in modes:
                flags = flags | lgpio.SET_OPEN_DRAIN
            if 'ACTIVE_LOW' in modes:
                flags = flags | lgpio.SET_ACTIVE_LOW
        
        if pin not in self.output_pins:
            claim_result = lgpio.gpio_claim_output(self.chip_handler, pin, lFlags=flags)
            if claim_result < 0:
                raise RuntimeError(f'GPIO "{pin}" output setup error: {claim_result}') 
            self.output_pins[pin] = claim_result
   
    def destructor(self):
        self.destroy_service(self.srv_read_bit)
        self.destroy_service(self.srv_write_bit)
        self.destroy_service(self.srv_write_pwm)

        # Frees used pind
        used_pins = [*self.input_pins.keys()] + [*self.output_pins.keys()]
        self.get_logger().info(f'Frees used GPIO: {used_pins}')
        for pin in used_pins:
            lgpio.gpio_free(self.chip_handler, pin)

        self.get_logger().info(f'Closing GPIO device {self.chip_handler}...')
        lgpio.gpiochip_close(self.chip_handler)

def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    gpio_node = GPIOServiceNode()
    executor.add_node(gpio_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException) as e:
        print(e)
    finally:
        gpio_node.destructor()
        gpio_node.destroy_node()

    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
