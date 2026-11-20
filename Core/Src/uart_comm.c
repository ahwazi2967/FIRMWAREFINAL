#include "uart_comm.h"
#include "ring.h"
#include "app.h"

extern UART_HandleTypeDef huart1;

static uint8_t rx_storage[RX_RING_SIZE];
static uint8_t tx_storage[TX_RING_SIZE];
static Ring    rx_ring;
static Ring    tx_ring;

static uint8_t           rx_byte;
static volatile uint8_t  tx_busy;
static volatile uint16_t tx_chunk_len;
void uart_comm_init(void) {
    ring_init(&rx_ring, rx_storage, RX_RING_SIZE);
    ring_init(&tx_ring, tx_storage, TX_RING_SIZE);
    tx_busy = 0;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        g_diag.rx_bytes++;
        if (!ring_push(&rx_ring, rx_byte)) {
            g_diag.rx_overflow++;
            g_app.flags |= FLAG_RX_OVERFLOW;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        g_diag.uart_errors++;
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

uint16_t uart_rx_available(void) {
    return ring_count(&rx_ring);
}

uint8_t uart_read_byte(uint8_t *byte) {
    return ring_pop(&rx_ring, byte);
}

static void uart_tx_kick(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if ((tx_busy == 0U) && (tx_ring.head != tx_ring.tail)) {
        uint16_t tail = tx_ring.tail;
        uint16_t head = tx_ring.head;
        uint16_t len = (head > tail) ? (uint16_t)(head - tail)
                                     : (uint16_t)(tx_ring.size - tail);
        tx_chunk_len = len;
        tx_busy = 1U;
        HAL_UART_Transmit_IT(&huart1, &tx_storage[tail], len);
    }

    __set_PRIMASK(primask);
}

uint8_t uart_send(const uint8_t *data, uint16_t len) {
    if (ring_free(&tx_ring) < len) {          // tidak muat: buang SELURUH frame
        g_diag.tx_drops++;
        return 0U;
    }
    for (uint16_t i = 0; i < len; i++) {
        ring_push(&tx_ring, data[i]);
    }
    uart_tx_kick();
    return 1U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        tx_ring.tail = (uint16_t)((tx_ring.tail + tx_chunk_len) % tx_ring.size);
        tx_busy = 0U;
        uart_tx_kick();                       // masih ada data? kirim potongan berikutnya
    }
}
