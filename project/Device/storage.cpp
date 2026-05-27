#include "Storage.h"
#include "tuning_config.h"
#include "zf_common_headfile.h"
#include <string.h>

// 选用倒数第一个扇区的倒数第一页 (规避与代码存储区冲突)
#define FLASH_SECTION_INDEX       (127)                                         
#define FLASH_PAGE_INDEX          (FLASH_PAGE_7)                                

// 魔数校验码 (用于判定 Flash 里存的是不是我们的参数)
#define CONFIG_MAGIC_WORD         (0xAA55CC55)


void Storage::init() {
    flash_init();
    
    // 如果该页有数据，尝试加载
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX)) {
        load_params();
    } else {
        // 如果完全是空的，将代码里自带的默认参数存进去打底
        save_params();
    }
}

void Storage::save_params() {
    flash_buffer_clear(); 
    
    // 在缓冲区的第 0 位写入魔数
    flash_union_buffer[0].uint32_type = CONFIG_MAGIC_WORD;
    
    // 将整个 tune 结构体内存直接 Copy 到缓冲区
    memcpy(&flash_union_buffer[1], &tune, sizeof(TuningConfig));
    
    // 擦除并写入
    flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
}

void Storage::load_params() {
    // 将数据从 flash 读取到逐飞缓冲区
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    
    // 只有当第一位的魔数匹配时，才允许覆盖 tune
    if(flash_union_buffer[0].uint32_type == CONFIG_MAGIC_WORD) {
        memcpy(&tune, &flash_union_buffer[1], sizeof(TuningConfig));
    }
}