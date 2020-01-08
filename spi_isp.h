#ifndef SPI_ISP_H_
#define SPI_ISP_H_

#include <stdint.h>

#include "avrdevice.h"

/* *********************************************************************** */

#define SPI_SET_CLK_MAX         0
#define SPI_SET_CLK_DEC         1

#define CMD_PROG_ENABLE_1       0xAC
#define CMD_PROG_ENABLE_2       0x53
#define CMD_CHIP_ERASE_1        0xAC
#define CMD_CHIP_ERASE_2        0x80
#define CMD_POLL_BUSY_1         0xF0 /* not used */
#define CMD_POLL_BUSY_2         0x00 /* not used */

#define CMD_LOAD_EADDR_1        0x4D /* not used */
#define CMD_LOAD_EADDR_2        0x00 /* not used */
#define CMD_LOAD_FLASH_HI       0x48
#define CMD_LOAD_FLASH_LO       0x40
#define CMD_LOAD_EEPROM_PAGE    0xC1 /* not used */

#define CMD_READ_FLASH_LO       0x20
#define CMD_READ_FLASH_HI       0x28
#define CMD_READ_EEPROM         0xA0
#define CMD_READ_LOCK_1         0x58
#define CMD_READ_LOCK_2         0x00
#define CMD_READ_SIG_1          0x30
#define CMD_READ_SIG_2          0x00
#define CMD_READ_FUSE_1         0x50
#define CMD_READ_FUSE_2         0x00
#define CMD_READ_FUSE_H_1       0x58
#define CMD_READ_FUSE_H_2       0x08
#define CMD_READ_FUSE_E_1       0x50
#define CMD_READ_FUSE_E_2       0x08
#define CMD_READ_CAL            0x38 /* not used */

#define CMD_WRITE_FLASH_PAGE    0x4C
#define CMD_WRITE_EEPROM        0xC0
#define CMD_WRITE_EEPROM_PAGE   0xC2 /* not used */
#define CMD_WRITE_LOCK_1        0xAC
#define CMD_WRITE_LOCK_2        0xE0
#define CMD_WRITE_FUSE_1        0xAC
#define CMD_WRITE_FUSE_2        0xA0
#define CMD_WRITE_FUSE_H_1      0xAC /* not used */
#define CMD_WRITE_FUSE_H_2      0xA8 /* not used */
#define CMD_WRITE_FUSE_E_1      0xAC /* not used */
#define CMD_WRITE_FUSE_E_2      0xA4 /* not used */

/* *********************************************************************** */

void    spi_set_clk         (uint8_t mode);
void    spi_init            (uint8_t enable);

uint8_t isp_enter_progmode  (void);
void    isp_mem_write       (uint8_t cmd, uint16_t addr, uint8_t val);
uint8_t isp_mem_read        (uint8_t cmd, uint16_t addr);
void    isp_mem_poll        (const avr_device_t * p_device);
void    isp_mem_pagewrite   (void);
void    isp_read_signature  (uint8_t * p_signature);
uint8_t isp_cmd3            (uint8_t cmd1, uint8_t cmd2,
                             uint8_t cmd3);
uint8_t isp_cmd4            (uint8_t cmd1, uint8_t cmd2,
                             uint8_t cmd3, uint8_t cmd4);

/* *********************************************************************** */

#endif /* SPI_ISP_H_ */
