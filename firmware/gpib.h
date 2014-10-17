/*
* GPIBUSB Adapter
* gpib.h
**
* © 2014 Steven Casagrande (scasagrande@galvant.ca).
*
* This file is a part of the GPIBUSB Adapter project.
* Licensed under the AGPL version 3.
**
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

uint32_t gpib_cmd(uint8_t *bytes, uint32_t length);
uint32_t gpib_write(uint8_t *bytes, uint32_t length, bool use_eoi);
uint32_t _gpib_write(uint8_t *bytes, uint32_t length, bool attention, bool use_eoi);
uint32_t gpib_read_byte(uint8_t *byte, bool *eoi_status);
uint32_t gpib_read(bool use_eoi, uint8_t eos_code, bool eot_enable, uint8_t eot_char);
uint32_t address_target_listen(uint32_t address);
uint32_t gpib_controller_assign(uint32_t address);

#define DIO_PORT GPIOB
#define CONTROL_PORT GPIOC
#define FLOW_PORT GPIOA

#define REN GPIO1
#define EOI GPIO2
#define DAV GPIO3
#define NRFD GPIO4
#define NDAC GPIO5
#define ATN GPIO6
#define SRQ GPIO7
#define IFC GPIO8

#define SC GPIO5
#define TE GPIO6
#define PE GPIO7
#define DC GPIO8

#define CMD_DCL 0x14
#define CMD_UNL 0x3f
#define CMD_UNT 0x5f
#define CMD_GET 0x8
#define CMD_SDC 0x04
#define CMD_LLO 0x11
#define CMD_GTL 0x1
#define CMD_SPE 0x18
#define CMD_SPD 0x19
