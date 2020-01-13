#ifndef TWI_MASTER_H_
#define TWI_MASTER_H_

#include <stdint.h>

/* *********************************************************************** */

/* SLA+R */
#define CMD_WAIT                0x00
#define CMD_READ_VERSION        0x01
#define CMD_READ_MEMORY         0x02

/* SLA+W */
#define CMD_SWITCH_APPLICATION  CMD_READ_VERSION
#define CMD_WRITE_MEMORY        CMD_READ_MEMORY

/* CMD_SWITCH_APPLICATION parameter */
#define BOOTTYPE_BOOTLOADER     0x00
#define BOOTTYPE_APPLICATION    0x80

/* CMD_{READ|WRITE}_* parameter */
#define MEMTYPE_CHIPINFO        0x00
#define MEMTYPE_FLASH           0x01
#define MEMTYPE_EEPROM          0x02

#define TWI_SUCCESS             0x00
#define TWI_ERROR               0x01

typedef struct twi_chipinfo_s
{
    uint8_t sig[3];
    uint8_t page_size;
    uint16_t flash_size;
    uint16_t eeprom_size;
} twi_chipinfo_t;

/* *********************************************************************** */

uint8_t twi_generic             (uint8_t twi_addr,
                                 const uint8_t * p_wr_data, uint16_t write_size,
                                 uint8_t * p_rd_data, uint16_t read_size);

uint8_t twi_switch_application  (uint8_t addr, uint8_t app);

uint8_t twi_read_version        (uint8_t addr, char * p_version,
                                 uint8_t version_length);

uint8_t twi_read_chipinfo       (uint8_t addr, twi_chipinfo_t * p_chipinfo);

uint8_t twi_read_memory         (uint8_t twi_addr, uint8_t memory_type,
                                 uint16_t memory_addr,
                                 uint8_t * p_data, uint16_t data_length);

uint8_t twi_write_memory        (uint8_t twi_addr, uint8_t memory_type,
                                 uint16_t memory_addr,
                                 const uint8_t * p_data, uint16_t data_length);

void twi_init                   (uint8_t enable);

/* *********************************************************************** */

#endif /* TWI_MASTER_H_ */
