#ifndef VIZZINI_H
#define VIZZINI_H

#define XR_SET_REG              0
#define XR_GETN_REG             1

#define UART_0_REG_BLOCK        0
#define UART_1_REG_BLOCK        1
#define UART_2_REG_BLOCK        2
#define UART_3_REG_BLOCK        3
#define URM_REG_BLOCK           4
#define PRM_REG_BLOCK           5
#define EPMERR_REG_BLOCK        6
#define RAMCTL_REG_BLOCK        0x64
#define TWI_ROM_REG_BLOCK       0x65
#define EPLOCALS_REG_BLOCK      0x66

#define MEM_SHADOW_REG_SIZE_S   5
#define MEM_SHADOW_REG_SIZE     (1 << MEM_SHADOW_REG_SIZE_S)

#define MEM_EP_LOCALS_SIZE_S    3
#define MEM_EP_LOCALS_SIZE      (1 << MEM_EP_LOCALS_SIZE_S)

#define EP_WIDE_MODE            0x03


#define UART_GPIO_MODE                                     0x01a

#define UART_GPIO_MODE_SEL_M                               0x7
#define UART_GPIO_MODE_SEL_S                               0
#define UART_GPIO_MODE_SEL                                 0x007

#define UART_GPIO_MODE_SEL_GPIO                            (0x0 << UART_GPIO_MODE_SEL_S)
#define UART_GPIO_MODE_SEL_RTS_CTS                         (0x1 << UART_GPIO_MODE_SEL_S)
#define UART_GPIO_MODE_SEL_DTR_DSR                         (0x2 << UART_GPIO_MODE_SEL_S)
#define UART_GPIO_MODE_SEL_XCVR_EN_ACT                     (0x3 << UART_GPIO_MODE_SEL_S)
#define UART_GPIO_MODE_SEL_XCVR_EN_FLOW                    (0x4 << UART_GPIO_MODE_SEL_S)

#define UART_GPIO_MODE_XCVR_EN_POL_M                       0x1
#define UART_GPIO_MODE_XCVR_EN_POL_S                       3
#define UART_GPIO_MODE_XCVR_EN_POL                         0x008

#define UART_ENABLE                                        0x003
#define UART_ENABLE_TX_M                                   0x1
#define UART_ENABLE_TX_S                                   0
#define UART_ENABLE_TX                                     0x001
#define UART_ENABLE_RX_M                                   0x1
#define UART_ENABLE_RX_S                                   1
#define UART_ENABLE_RX                                     0x002

#define UART_CLOCK_DIVISOR_0                               0x004
#define UART_CLOCK_DIVISOR_1                               0x005
#define UART_CLOCK_DIVISOR_2                               0x006

#define UART_CLOCK_DIVISOR_2_MSB_M                         0x7
#define UART_CLOCK_DIVISOR_2_MSB_S                         0
#define UART_CLOCK_DIVISOR_2_MSB                           0x007
#define UART_CLOCK_DIVISOR_2_DIAGMODE_M                    0x1
#define UART_CLOCK_DIVISOR_2_DIAGMODE_S                    3
#define UART_CLOCK_DIVISOR_2_DIAGMODE                      0x008

#define UART_TX_CLOCK_MASK_0                               0x007
#define UART_TX_CLOCK_MASK_1                               0x008

#define UART_RX_CLOCK_MASK_0                               0x009
#define UART_RX_CLOCK_MASK_1                               0x00a

#define UART_FORMAT                                        0x00b

#define UART_FORMAT_SIZE_M                                 0xf
#define UART_FORMAT_SIZE_S                                 0
#define UART_FORMAT_SIZE                                   0x00f

#define UART_FORMAT_SIZE_7                                 (0x7 << UART_FORMAT_SIZE_S)
#define UART_FORMAT_SIZE_8                                 (0x8 << UART_FORMAT_SIZE_S)
#define UART_FORMAT_SIZE_9                                 (0x9 << UART_FORMAT_SIZE_S)

#define UART_FORMAT_PARITY_M                               0x7
#define UART_FORMAT_PARITY_S                               4
#define UART_FORMAT_PARITY                                 0x070

#define UART_FORMAT_PARITY_NONE                            (0x0 << UART_FORMAT_PARITY_S)
#define UART_FORMAT_PARITY_ODD                             (0x1 << UART_FORMAT_PARITY_S)
#define UART_FORMAT_PARITY_EVEN                            (0x2 << UART_FORMAT_PARITY_S)
#define UART_FORMAT_PARITY_1                               (0x3 << UART_FORMAT_PARITY_S)
#define UART_FORMAT_PARITY_0                               (0x4 << UART_FORMAT_PARITY_S)

#define UART_FORMAT_STOP_M                                 0x1
#define UART_FORMAT_STOP_S                                 7
#define UART_FORMAT_STOP                                   0x080

#define UART_FORMAT_STOP_1                                 (0x0 << UART_FORMAT_STOP_S)
#define UART_FORMAT_STOP_2                                 (0x1 << UART_FORMAT_STOP_S)

#define UART_FORMAT_MODE_7N1                               0
#define UART_FORMAT_MODE_RES1                              1
#define UART_FORMAT_MODE_RES2                              2
#define UART_FORMAT_MODE_RES3                              3
#define UART_FORMAT_MODE_7N2                               4
#define UART_FORMAT_MODE_7P1                               5
#define UART_FORMAT_MODE_8N1                               6
#define UART_FORMAT_MODE_RES7                              7
#define UART_FORMAT_MODE_7P2                               8
#define UART_FORMAT_MODE_8N2                               9
#define UART_FORMAT_MODE_8P1                               10
#define UART_FORMAT_MODE_9N1                               11
#define UART_FORMAT_MODE_8P2                               12
#define UART_FORMAT_MODE_RESD                              13
#define UART_FORMAT_MODE_RESE                              14
#define UART_FORMAT_MODE_9N2                               15

#define UART_FLOW                                          0x00c

#define UART_FLOW_MODE_M                                   0x7
#define UART_FLOW_MODE_S                                   0
#define UART_FLOW_MODE                                     0x007

#define UART_FLOW_MODE_NONE                                (0x0 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_HW                                  (0x1 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_SW                                  (0x2 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_ADDR_MATCH                          (0x3 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_ADDR_MATCH_TX                       (0x4 << UART_FLOW_MODE_S)

#define UART_FLOW_HALF_DUPLEX_M                            0x1
#define UART_FLOW_HALF_DUPLEX_S                            3
#define UART_FLOW_HALF_DUPLEX                              0x008

#define UART_LOOPBACK_CTL                                  0x012
#define UART_LOOPBACK_CTL_ENABLE_M                         0x1
#define UART_LOOPBACK_CTL_ENABLE_S                         2
#define UART_LOOPBACK_CTL_ENABLE                           0x004
#define UART_LOOPBACK_CTL_RX_SOURCE_M                      0x3
#define UART_LOOPBACK_CTL_RX_SOURCE_S                      0
#define UART_LOOPBACK_CTL_RX_SOURCE                        0x003
#define UART_LOOPBACK_CTL_RX_UART0                         (0x0 << UART_LOOPBACK_CTL_RX_SOURCE_S)
#define UART_LOOPBACK_CTL_RX_UART1                         (0x1 << UART_LOOPBACK_CTL_RX_SOURCE_S)
#define UART_LOOPBACK_CTL_RX_UART2                         (0x2 << UART_LOOPBACK_CTL_RX_SOURCE_S)
#define UART_LOOPBACK_CTL_RX_UART3                         (0x3 << UART_LOOPBACK_CTL_RX_SOURCE_S)

#define UART_CHANNEL_NUM                                   0x00d

#define UART_XON_CHAR                                      0x010
#define UART_XOFF_CHAR                                     0x011

#define UART_GPIO_SET                                      0x01d
#define UART_GPIO_CLR                                      0x01e
#define UART_GPIO_STATUS                                   0x01f

#define URM_ENABLE_BASE                                    0x010
#define URM_ENABLE_0                                       0x010
#define URM_ENABLE_0_TX_M                                  0x1
#define URM_ENABLE_0_TX_S                                  0
#define URM_ENABLE_0_TX                                    0x001
#define URM_ENABLE_0_RX_M                                  0x1
#define URM_ENABLE_0_RX_S                                  1
#define URM_ENABLE_0_RX                                    0x002

#define URM_RX_FIFO_RESET_0                                0x018
#define URM_RX_FIFO_RESET_1                                0x019
#define URM_RX_FIFO_RESET_2                                0x01a
#define URM_RX_FIFO_RESET_3                                0x01b
#define URM_TX_FIFO_RESET_0                                0x01c
#define URM_TX_FIFO_RESET_1                                0x01d
#define URM_TX_FIFO_RESET_2                                0x01e
#define URM_TX_FIFO_RESET_3                                0x01f


#define RAMCTL_REGS_TXFIFO_0_LEVEL                         0x000
#define RAMCTL_REGS_TXFIFO_1_LEVEL                         0x001
#define RAMCTL_REGS_TXFIFO_2_LEVEL                         0x002
#define RAMCTL_REGS_TXFIFO_3_LEVEL                         0x003
#define RAMCTL_REGS_RXFIFO_0_LEVEL                         0x004

#define RAMCTL_REGS_RXFIFO_0_LEVEL_LEVEL_M                 0x7ff
#define RAMCTL_REGS_RXFIFO_0_LEVEL_LEVEL_S                 0
#define RAMCTL_REGS_RXFIFO_0_LEVEL_LEVEL                   0x7ff
#define RAMCTL_REGS_RXFIFO_0_LEVEL_STALE_M                 0x1
#define RAMCTL_REGS_RXFIFO_0_LEVEL_STALE_S                 11
#define RAMCTL_REGS_RXFIFO_0_LEVEL_STALE                   0x800

#define RAMCTL_REGS_RXFIFO_1_LEVEL                         0x005
#define RAMCTL_REGS_RXFIFO_2_LEVEL                         0x006
#define RAMCTL_REGS_RXFIFO_3_LEVEL                         0x007

#define RAMCTL_BUFFER_PARITY                               0x1
#define RAMCTL_BUFFER_BREAK                                0x2
#define RAMCTL_BUFFER_FRAME                                0x4
#define RAMCTL_BUFFER_OVERRUN                              0x8

#endif /*VIZZINI_H*/
