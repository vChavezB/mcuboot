#include <stdio.h>
#include <zephyr/drivers/can.h>
#include <assert.h>
#include <string.h>
#include <zephyr/kernel.h>
#include "bootutil/bootutil_log.h"

BOOT_LOG_MODULE_REGISTER(can_adapter);

#if defined(CONFIG_BOOT_CAN) && \
    (!DT_HAS_CHOSEN(zephyr_can_mcumgr))
#error Zephyr CAN MCU manager is required for CAN recovery
#endif


static const struct can_filter smp_rx_filter = {
	.id = CONFIG_MCUBOOT_CAN_RX_ID,
	.mask =  CONFIG_MCUBOOT_CAN_RX_ID > CAN_STD_ID_MASK ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK,
	.flags = CONFIG_MCUBOOT_CAN_RX_ID > CAN_STD_ID_MASK ? CAN_FILTER_IDE: 0
};

static struct can_frame frame;

static struct device const *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_can_mcumgr));

/** @brief Console input representation
 *
 * This struct is used to represent an input line from a serial interface.
 */
struct line_input {
	/** Required to use sys_slist */
	sys_snode_t node;

	int len;
	/** Buffer where the input line is recorded */
	char line[CONFIG_BOOT_MAX_LINE_INPUT_LEN];
};

static struct device const *can_dev;
static struct line_input line_bufs[CONFIG_BOOT_LINE_BUFS];

static sys_slist_t avail_queue;
static sys_slist_t lines_queue;

static uint16_t cur;
static struct line_input *cmd;

void can_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data) {
	if (frame->id == CONFIG_MCUBOOT_CAN_RX_ID) {
		if (!cmd) {
			sys_snode_t *node;

			node = sys_slist_get(&avail_queue);
			if (!node) {
				BOOT_LOG_ERR("Not enough memory to store"
					     " incoming data!");
				return;
			}
			cmd = CONTAINER_OF(node, struct line_input, node);
		}

		for (int i = 0; i < frame->dlc; i++) {
			char byte = frame->data[i];

			if (cur < CONFIG_BOOT_MAX_LINE_INPUT_LEN) {
				cmd->line[cur++] = byte;
			}
			if (byte ==  '\n') {
				cmd->len = cur;
				sys_slist_append(&lines_queue, &cmd->node);
				cur = 0;
				cmd = NULL;
			}
		}	
	}
}

static int
boot_can_fifo_getline(char **line)
{
	static struct line_input *cmd;
	sys_snode_t *node;
	int key;

	key = irq_lock();
	/* Recycle cmd buffer returned previous time */
	if (cmd != NULL) {
		if (sys_slist_peek_tail(&avail_queue) != &cmd->node) {
			sys_slist_append(&avail_queue, &cmd->node);
		}
	}

	node = sys_slist_get(&lines_queue);
	irq_unlock(key);

	if (node == NULL) {
		cmd = NULL;
		*line = NULL;

		return 0;
	}

	cmd = CONTAINER_OF(node, struct line_input, node);
	*line = cmd->line;
	return cmd->len;
}

void
console_write(const char *str, int cnt)
{
	int i;
	int eof_idx = -1;
	for (i = 0; i < cnt; i++) {
		if (str[i] == EOF)  {
			eof_idx = i;
			break;
		}
	}
	if (eof_idx != -1) {
		cnt = eof_idx;
	}

	while(cnt > 0) {
		int len = cnt > CAN_MAX_DLC ? CAN_MAX_DLC : cnt;
		frame.id = CONFIG_MCUBOOT_CAN_TX_ID;
		frame.dlc = len;
		frame.flags = CONFIG_MCUBOOT_CAN_TX_ID > CAN_STD_ID_MASK ? CAN_FILTER_IDE: 0;
		memcpy(frame.data, str, len);
		can_send(can_dev, &frame, K_FOREVER, NULL, NULL);
		str += len;
		cnt -= len;
	}
}

int
console_read(char *str, int str_size, int *newline)
{
	char *line;
	int len;

	len = boot_can_fifo_getline(&line);

	if (line == NULL) {
		*newline = 0;
		return 0;
	}

	if (len > str_size - 1) {
		len = str_size - 1;
	}

	memcpy(str, line, len);
	str[len] = '\0';
	*newline = 1;
	return len + 1;
}

int
boot_console_init(void)
{
	int i;

	/* Zephyr CAN handler takes an empty buffer from avail_queue,
	 * stores CAN input in it until EOL, and then puts it into
	 * lines_queue.
	 */
	sys_slist_init(&avail_queue);
	sys_slist_init(&lines_queue);

	for (i = 0; i < ARRAY_SIZE(line_bufs); i++) {
		sys_slist_append(&avail_queue, &line_bufs[i].node);
	}

	int rc = can_start(can_dev);
	if (rc !=0) {
		return rc;
	}

	rc = can_add_rx_filter(can_dev, can_rx_cb, NULL, &smp_rx_filter);

	if (rc < 0 ) {
		rc = -EINVAL;
	}
	return rc;
}
