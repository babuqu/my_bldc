#include "main.h"
#include "param_registry.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>

/* 参数日志使用 Sector 10 和 11；链接区域必须避开这两个扇区。 */
#define PARAM_STORAGE_SECTOR_A       FLASH_SECTOR_10
#define PARAM_STORAGE_SECTOR_B       FLASH_SECTOR_11
#define PARAM_STORAGE_ADDRESS_A      0x080C0000u
#define PARAM_STORAGE_ADDRESS_B      0x080E0000u
#define PARAM_STORAGE_SECTOR_SIZE    0x00020000u
#define PARAM_STORAGE_SLOT_SIZE      128u
#define PARAM_STORAGE_MAGIC          0x4D524150u
#define PARAM_STORAGE_VERSION        2u
#define PARAM_STORAGE_HEADER_SIZE    16u
#define PARAM_STORAGE_RECORD_SIZE    10u
#define PARAM_STORAGE_CRC_SIZE       4u
#define PARAM_STORAGE_SLOTS_PER_SECTOR \
    (PARAM_STORAGE_SECTOR_SIZE / PARAM_STORAGE_SLOT_SIZE)

/* 保存扫描到的最新有效快照位置和序列号。 */
typedef struct
{
    uint8_t found;
    uint8_t sector;
    uint16_t slot;
    uint32_t sequence;
} param_storage_latest_t;

static const uint32_t g_sector_addresses[2] =
{
    PARAM_STORAGE_ADDRESS_A,
    PARAM_STORAGE_ADDRESS_B
};

static const uint32_t g_sector_numbers[2] =
{
    PARAM_STORAGE_SECTOR_A,
    PARAM_STORAGE_SECTOR_B
};

static uint8_t g_storage_ready;
static uint8_t g_active_sector;
static uint16_t g_next_slot;
static uint32_t g_latest_sequence;

static uint8_t ParamStorage_FindErasedSlot(uint8_t sector, uint16_t *slot_index);

/* 按小端序读写字节，避免依赖结构体对齐和填充。 */
static uint16_t ParamStorage_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t ParamStorage_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void ParamStorage_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void ParamStorage_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static uint16_t ParamStorage_Crc16(const uint8_t *data, uint16_t length)
{
    /* 计算 CRC-16/Modbus，初值为 0xFFFF，多项式为 0xA001。 */
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t bit;

    for (i = 0u; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0u; bit < 8u; ++bit)
        {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u)
                             : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* 计算快照头和所有参数记录的 CRC32。 */
static uint32_t ParamStorage_Crc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint16_t i;
    uint8_t bit;

    for (i = 0u; i < length; ++i)
    {
        crc ^= data[i];
        for (bit = 0u; bit < 8u; ++bit)
        {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static uint8_t ParamStorage_IsErased(const uint8_t *slot)
{
    /* Flash 擦除后的每个字节均为 0xFF。 */
    uint16_t i;
    for (i = 0u; i < PARAM_STORAGE_SLOT_SIZE; ++i)
    {
        if (slot[i] != 0xFFu)
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t ParamStorage_ValidateSlot(const uint8_t *slot, uint32_t *sequence)
{
    uint16_t count;
    uint16_t expected_count;
    uint32_t payload_length;
    uint32_t crc_offset;
    uint32_t record_offset;
    uint16_t i;

    /* 检查标识、版本、记录数量、整包 CRC 和每条记录。 */
    if (slot == NULL || sequence == NULL ||
        ParamStorage_ReadU32(slot) != PARAM_STORAGE_MAGIC ||
        ParamStorage_ReadU16(&slot[4]) != PARAM_STORAGE_VERSION)
    {
        return 0u;
    }

    count = ParamStorage_ReadU16(&slot[6]);
    expected_count = ParamRegistry_Count();
    payload_length = ParamStorage_ReadU32(&slot[12]);
    if (count != expected_count ||
        payload_length != ((uint32_t)count * PARAM_STORAGE_RECORD_SIZE))
    {
        return 0u;
    }

    crc_offset = PARAM_STORAGE_HEADER_SIZE + payload_length;
    if ((crc_offset + PARAM_STORAGE_CRC_SIZE) > PARAM_STORAGE_SLOT_SIZE ||
        ParamStorage_ReadU32(&slot[crc_offset]) !=
            ParamStorage_Crc32(slot, (uint16_t)crc_offset))
    {
        return 0u;
    }

    record_offset = PARAM_STORAGE_HEADER_SIZE;
    for (i = 0u; i < count; ++i)
    {
        uint16_t record_crc = ParamStorage_ReadU16(&slot[record_offset + 8u]);
        if (record_crc != ParamStorage_Crc16(&slot[record_offset], 8u) ||
            ParamValue_ValidateRecord(&slot[record_offset], 8u) != 0)
        {
            return 0u;
        }
        record_offset += PARAM_STORAGE_RECORD_SIZE;
    }

    *sequence = ParamStorage_ReadU32(&slot[8]);
    return 1u;
}
static uint8_t ParamStorage_IsNewer(uint32_t candidate, uint32_t reference)
{
    /* 使用有符号差值比较序列号，兼容 uint32_t 回绕。 */
    return ((int32_t)(candidate - reference) > 0) ? 1u : 0u;
}

static param_storage_latest_t ParamStorage_FindLatest(uint8_t *snapshot)
{
    param_storage_latest_t latest = {0u, 0u, 0u, 0u};
    uint8_t sector;
    uint16_t slot_index;

    /* 扫描两个扇区，选择 CRC 正确且序列号最新的快照。 */
    for (sector = 0u; sector < 2u; ++sector)
    {
        for (slot_index = 0u;
             ((uint32_t)(slot_index + 1u) * PARAM_STORAGE_SLOT_SIZE) <= PARAM_STORAGE_SECTOR_SIZE;
             ++slot_index)
        {
            const uint8_t *slot = (const uint8_t *)(g_sector_addresses[sector] +
                                  ((uint32_t)slot_index * PARAM_STORAGE_SLOT_SIZE));
            uint32_t sequence;
            if (ParamStorage_ValidateSlot(slot, &sequence) &&
                (!latest.found || ParamStorage_IsNewer(sequence, latest.sequence)))
            {
                latest.found = 1u;
                latest.sector = sector;
                latest.slot = slot_index;
                latest.sequence = sequence;
                memcpy(snapshot, slot, PARAM_STORAGE_SLOT_SIZE);
            }
        }
    }
    return latest;
}
static uint8_t ParamStorage_ApplySnapshot(const uint8_t *snapshot)
{
    uint16_t count = ParamStorage_ReadU16(&snapshot[6]);
    uint16_t i;
    uint32_t offset = PARAM_STORAGE_HEADER_SIZE;

    /* 快照整体校验通过后，再逐条更新 registry 当前值。 */
    for (i = 0u; i < count; ++i)
    {
        if (ParamValue_ParseRecord(&snapshot[offset], 8u) != 0)
        {
            return 0u;
        }
        offset += PARAM_STORAGE_RECORD_SIZE;
    }
    return 1u;
}

uint8_t ParamStorage_Load(void)
{
    uint8_t snapshot[PARAM_STORAGE_SLOT_SIZE];
    param_storage_latest_t latest;

    /* 初始化参数默认值，再查找 Flash 中最新的有效快照。 */
    if (ParamRegistry_Init() != 0)
    {
        return 1u;
    }
    latest = ParamStorage_FindLatest(snapshot);
    if (latest.found && !ParamStorage_ApplySnapshot(snapshot))
    {
        g_storage_ready = 0u;
        return 1u;
    }

    /* 建立追加写入位置，后续保存时无需重新扫描整个 Flash。 */
    g_active_sector = latest.found ? latest.sector : 0u;
    g_latest_sequence = latest.found ? latest.sequence : 0u;
    if (!ParamStorage_FindErasedSlot(g_active_sector, &g_next_slot))
    {
        g_next_slot = PARAM_STORAGE_SLOTS_PER_SECTOR;
    }
    g_storage_ready = 1u;
    return 0u;
}

static uint8_t ParamStorage_BuildSnapshot(uint8_t *snapshot, uint32_t sequence,
                                         uint16_t *program_length)
{
    uint16_t count = ParamRegistry_Count();
    uint16_t i;
    uint16_t written;
    uint32_t offset = PARAM_STORAGE_HEADER_SIZE;
    uint32_t crc_offset;
    uint8_t record[8];

    /* 快照头包含标识、版本、记录数量、序列号和负载长度。 */
    memset(snapshot, 0xFF, PARAM_STORAGE_SLOT_SIZE);
    ParamStorage_WriteU32(snapshot, PARAM_STORAGE_MAGIC);
    ParamStorage_WriteU16(&snapshot[4], PARAM_STORAGE_VERSION);
    ParamStorage_WriteU16(&snapshot[6], count);
    ParamStorage_WriteU32(&snapshot[8], sequence);
    ParamStorage_WriteU32(&snapshot[12], (uint32_t)count * PARAM_STORAGE_RECORD_SIZE);

    /* 每条参数记录包含 ID、类型、长度、数据和 CRC16。 */
    for (i = 0u; i < count; ++i)
    {
        const param_desc_t *desc = ParamRegistry_GetAt(i);
        if (desc == NULL ||
            ParamValue_BuildRecord(desc->id, record, sizeof(record), &written) != 0 ||
            written != 8u)
        {
            return 0u;
        }
        memcpy(&snapshot[offset], record, written);
        ParamStorage_WriteU16(&snapshot[offset + written],
                              ParamStorage_Crc16(record, written));
        offset += PARAM_STORAGE_RECORD_SIZE;
    }

    /* CRC32 位于记录之后；写入长度按 32 位字对齐。 */
    crc_offset = offset;
    ParamStorage_WriteU32(&snapshot[crc_offset],
                           ParamStorage_Crc32(snapshot, (uint16_t)crc_offset));
    *program_length = (uint16_t)((crc_offset + PARAM_STORAGE_CRC_SIZE + 3u) & ~3u);
    return (*program_length <= PARAM_STORAGE_SLOT_SIZE) ? 1u : 0u;
}

static uint8_t ParamStorage_FindErasedSlot(uint8_t sector, uint16_t *slot_index)
{
    uint16_t i;
    /* 鏌ユ壘鎵囧尯涓涓€涓畬鏁存摝闄ょ殑妲戒綅銆?*/
    for (i = 0u;
         ((uint32_t)(i + 1u) * PARAM_STORAGE_SLOT_SIZE) <= PARAM_STORAGE_SECTOR_SIZE;
         ++i)
    {
        const uint8_t *slot = (const uint8_t *)(g_sector_addresses[sector] +
                              ((uint32_t)i * PARAM_STORAGE_SLOT_SIZE));
        if (ParamStorage_IsErased(slot))
        {
            *slot_index = i;
            return 1u;
        }
    }
    return 0u;
}

static void ParamStorage_AdvancePastUsedSlots(uint8_t sector, uint16_t *slot_index)
{
    /* 断电可能留下未完成槽位；跳过该槽，避免重复写入。 */
    while (*slot_index < PARAM_STORAGE_SLOTS_PER_SECTOR)
    {
        const uint8_t *slot = (const uint8_t *)(g_sector_addresses[sector] +
                              ((uint32_t)(*slot_index) * PARAM_STORAGE_SLOT_SIZE));
        if (ParamStorage_IsErased(slot))
        {
            break;
        }
        ++(*slot_index);
    }
}

static uint8_t ParamStorage_EraseSector(uint8_t sector)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0u;

    /* 擦除指定参数扇区；切换时保留当前有效快照所在扇区。 */
    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = g_sector_numbers[sector];
    erase.NbSectors = 1u;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    return (HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK) ? 1u : 0u;
}

static uint8_t ParamStorage_ProgramSlot(uint8_t sector, uint16_t slot_index,
                                        const uint8_t *snapshot, uint16_t length)
{
    uint32_t address = g_sector_addresses[sector] +
                       ((uint32_t)slot_index * PARAM_STORAGE_SLOT_SIZE);
    uint32_t sequence;
    uint16_t offset;

    /* STM32F407 按 32 位字编程；完成后锁定 Flash 并校验槽位。 */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 0u;
    }
    for (offset = 0u; offset < length; offset += 4u)
    {
        uint32_t word = ParamStorage_ReadU32(&snapshot[offset]);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + offset, word) != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            return 0u;
        }
    }
    (void)HAL_FLASH_Lock();

    return ParamStorage_ValidateSlot((const uint8_t *)address, &sequence) ? 1u : 0u;
}

uint8_t ParamStorage_Save(void)
{
    uint8_t snapshot[PARAM_STORAGE_SLOT_SIZE];
    uint8_t sector;
    uint16_t slot_index;
    uint16_t program_length;
    uint32_t next_sequence;
    uint8_t needs_erase;
    uint32_t verified_sequence;

    /* 保存前确保 registry 已初始化，并在首次调用时扫描 Flash。 */
    if (ParamRegistry_Init() != 0)
    {
        return 0u;
    }
    if (!g_storage_ready && ParamStorage_Load() != 0u)
    {
        return 0u;
    }

    /* 当前扇区有空槽时追加；写满后切换到另一个扇区。 */
    next_sequence = g_latest_sequence + 1u;
    needs_erase = (g_next_slot >= PARAM_STORAGE_SLOTS_PER_SECTOR) ? 1u : 0u;
    sector = needs_erase ? (uint8_t)(1u - g_active_sector) : g_active_sector;
    slot_index = needs_erase ? 0u : g_next_slot;
    if (!ParamStorage_BuildSnapshot(snapshot, next_sequence, &program_length))
    {
        return 0u;
    }

    /* 切换扇区时先擦除非活动区；新快照验证成功前保留旧快照。 */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 0u;
    }
    if (needs_erase && !ParamStorage_EraseSector(sector))
    {
        (void)HAL_FLASH_Lock();
        return 0u;
    }
    (void)HAL_FLASH_Lock();

    /* 写入并回读校验成功后，再更新扇区、序列号和写入位置。 */
    if (!ParamStorage_ProgramSlot(sector, slot_index, snapshot, program_length) ||
        !ParamStorage_ValidateSlot((const uint8_t *)(g_sector_addresses[sector] +
                                  ((uint32_t)slot_index * PARAM_STORAGE_SLOT_SIZE)),
                                  &verified_sequence) || verified_sequence != next_sequence)
    {
        if (!needs_erase)
        {
            g_next_slot = (uint16_t)(slot_index + 1u);
            ParamStorage_AdvancePastUsedSlots(sector, &g_next_slot);
        }
        return 0u;
    }

    g_active_sector = sector;
    g_latest_sequence = next_sequence;
    g_next_slot = (uint16_t)(slot_index + 1u);
    ParamStorage_AdvancePastUsedSlots(sector, &g_next_slot);
    g_storage_ready = 1u;
    return 1u;
}
