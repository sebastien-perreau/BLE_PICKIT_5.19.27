#include "sdk_common.h"
#include "nrf_log.h"
#include "ble_pickit_board.h"
#include "ble_vsd.h"
#include "ble_pickit_service.h"


static ble_pickit_t * p_vsd;
static uint8_t current_id_requested = ID_NONE;

static void _boot(uint8_t *buffer);
static void _version(uint8_t *buffer);
static void _conn_status(uint8_t *buffer);
static void _ble_params(uint8_t *buffer);
static void _pa_lna_param(uint8_t *buffer);
static void _transfer_ble_to_uart(uint8_t *buffer);
static void _extended_transfer_ble_to_uart(uint8_t *buffer);
static void _notif_buffer(uint8_t *buffer);

static bool is_vsd_send_request_free_for_id(uint8_t id);
static uint8_t vsd_send_request(p_function ptr, bool is_extended_message, uint8_t id);

void ble_init(ble_pickit_t * p_vsd_params)
{
    p_vsd = p_vsd_params;
    p_vsd->flags.boot_mode = true;
}

void ble_stack_tasks()
{
	static uint64_t tick_blink_led_1 = 0;
	static uint64_t tick_blink_led_3 = 0;
	ret_code_t err_code;

	if (p_vsd->params.leds_status_enable)
	{
		if (!p_vsd->status.is_init_done)
		{
			board_led_set(LED_1);
			board_led_set(LED_2);
			board_led_set(LED_3);
		}
		else
		{

			if (p_vsd->status.is_ble_service_has_event)
			{
				tick_blink_led_1 = mGetTick();
				p_vsd->status.is_ble_service_has_event = false;
				board_led_set(LED_1);
			}
			else if (mTickCompare(tick_blink_led_1) > TICK_1MS)
			{
				board_led_clr(LED_1);
			}

			if (p_vsd->status.is_connected_to_a_central)
			{
				board_led_set(LED_2);
			}
			else if (p_vsd->status.is_in_advertising_mode)
			{
				board_led_lat(LED_2, (mGetTick() >> 12)&1);
			}
			else
			{
				board_led_clr(LED_2);
			}

			if ((p_vsd->uart.transmit_in_progress) || (p_vsd->status.is_uart_message_receives))
			{
				tick_blink_led_3 = mGetTick();
				p_vsd->status.is_uart_message_receives = false;
				board_led_set(LED_3);
			}
			else if (mTickCompare(tick_blink_led_3) > TICK_20MS)
			{
				board_led_clr(LED_3);
			}
		}
	}
	else
	{
		board_led_clr(LED_1);
		board_led_clr(LED_2);
		board_led_clr(LED_3);
	}

    err_code = app_uart_get(&p_vsd->uart.buffer[p_vsd->uart.index]);
	if (err_code == NRF_SUCCESS)
	{
		p_vsd->uart.receive_in_progress = true;
		p_vsd->uart.tick = mGetTick();
		p_vsd->uart.index++;
	}

    if (mTickCompare(p_vsd->uart.tick) >= TICK_300US)
    {
        if (	(p_vsd->uart.index == 3) && 		\
                (p_vsd->uart.buffer[0] == 'A') && 	\
                (p_vsd->uart.buffer[1] == 'C') && 	\
                (p_vsd->uart.buffer[2] == 'K'))
        {
            p_vsd->uart.message_type = UART_ACK_MESSAGE;
        }
        else if (	(p_vsd->uart.index == 4) &&	 		\
                    (p_vsd->uart.buffer[0] == 'N') && 	\
                    (p_vsd->uart.buffer[1] == 'A') && 	\
                    (p_vsd->uart.buffer[2] == 'C') && 	\
                    (p_vsd->uart.buffer[3] == 'K'))
        {
            p_vsd->uart.message_type = UART_NACK_MESSAGE;
        }
        else if ((p_vsd->uart.index > 5) && (p_vsd->uart.buffer[1] == 'W'))
        {
            p_vsd->uart.message_type = UART_NEW_MESSAGE;
        }
        else
        {
            p_vsd->uart.message_type = UART_OTHER_MESSAGE;
        }
        p_vsd->uart.index = 0;
        p_vsd->uart.receive_in_progress = false;
    }

    if (p_vsd->uart.message_type == UART_NEW_MESSAGE)
    {
        uint8_t i;
        uint16_t crc_calc, crc_uart;

        p_vsd->status.is_uart_message_receives = true;
        p_vsd->uart.message_type = UART_NO_MESSAGE;

        crc_calc = fu_crc_16_ibm(p_vsd->uart.buffer, p_vsd->uart.buffer[2]+3);
        crc_uart = (p_vsd->uart.buffer[p_vsd->uart.buffer[2]+3] << 8) + (p_vsd->uart.buffer[p_vsd->uart.buffer[2]+4] << 0);

        if (crc_calc == crc_uart)
        {
            p_vsd->incoming_uart_message.id = p_vsd->uart.buffer[0];
            p_vsd->incoming_uart_message.type = p_vsd->uart.buffer[1];
            p_vsd->incoming_uart_message.length = p_vsd->uart.buffer[2];
            for (i = 0 ; i < p_vsd->incoming_uart_message.length ; i++)
            {
                p_vsd->incoming_uart_message.data[i] = p_vsd->uart.buffer[3+i];
            }
            do {} while (app_uart_put('A') != NRF_SUCCESS);
			do {} while (app_uart_put('C') != NRF_SUCCESS);
			do {} while (app_uart_put('K') != NRF_SUCCESS);
            p_vsd->uart.transmit_in_progress = true;
        }
        else
        {
            p_vsd->incoming_uart_message.id = ID_NONE;
            do {} while (app_uart_put('N') != NRF_SUCCESS);
            do {} while (app_uart_put('A') != NRF_SUCCESS);
			do {} while (app_uart_put('C') != NRF_SUCCESS);
			do {} while (app_uart_put('K') != NRF_SUCCESS);
            p_vsd->uart.transmit_in_progress = true;
        }
        memset(p_vsd->uart.buffer, 0, sizeof(p_vsd->uart.buffer));

        switch (p_vsd->incoming_uart_message.id)
        {
        	case ID_PA_LNA:
        		p_vsd->params.pa_lna_enable = p_vsd->incoming_uart_message.data[0] & 0x01;
        		break;

        	case ID_LED_STATUS:
        		p_vsd->params.leds_status_enable = p_vsd->incoming_uart_message.data[0] & 0x01;
        		p_vsd->flags.send_ble_params = true;
				ble_pickit_parameters_notification_send();
        		break;

			case ID_SET_NAME:
				memcpy(p_vsd->infos.device_name, p_vsd->incoming_uart_message.data, p_vsd->incoming_uart_message.length);
				p_vsd->infos.device_name[p_vsd->incoming_uart_message.length] = '\0';
				break;

            case ID_GET_VERSION:
                p_vsd->flags.send_version = true;
                break;

            case ID_ADV_INTERVAL:
            	p_vsd->params.preferred_gap_params.adv_interval = (p_vsd->incoming_uart_message.data[0] << 8) | (p_vsd->incoming_uart_message.data[1] << 0);
            	break;

            case ID_ADV_TIMEOUT:
            	p_vsd->params.preferred_gap_params.adv_timeout = (p_vsd->incoming_uart_message.data[0] << 8) | (p_vsd->incoming_uart_message.data[1] << 0);
            	break;

            case ID_SET_BLE_CONN_PARAMS:
            	if (	(p_vsd->params.preferred_gap_params.conn_params.min_conn_interval != ((p_vsd->incoming_uart_message.data[0] << 8) | (p_vsd->incoming_uart_message.data[1] << 0))) ||
            			(p_vsd->params.preferred_gap_params.conn_params.max_conn_interval != ((p_vsd->incoming_uart_message.data[2] << 8) | (p_vsd->incoming_uart_message.data[3] << 0))) ||
						(p_vsd->params.preferred_gap_params.conn_params.slave_latency != ((p_vsd->incoming_uart_message.data[4] << 8) | (p_vsd->incoming_uart_message.data[5] << 0))) ||
						(p_vsd->params.preferred_gap_params.conn_params.conn_sup_timeout != ((p_vsd->incoming_uart_message.data[6] << 8) | (p_vsd->incoming_uart_message.data[7] << 0))))
				{
            		p_vsd->params.preferred_gap_params.conn_params.min_conn_interval = (p_vsd->incoming_uart_message.data[0] << 8) | (p_vsd->incoming_uart_message.data[1] << 0);
            		p_vsd->params.preferred_gap_params.conn_params.max_conn_interval = (p_vsd->incoming_uart_message.data[2] << 8) | (p_vsd->incoming_uart_message.data[3] << 0);
            		p_vsd->params.preferred_gap_params.conn_params.slave_latency = (p_vsd->incoming_uart_message.data[4] << 8) | (p_vsd->incoming_uart_message.data[5] << 0);
            		p_vsd->params.preferred_gap_params.conn_params.conn_sup_timeout = (p_vsd->incoming_uart_message.data[6] << 8) | (p_vsd->incoming_uart_message.data[7] << 0);
					p_vsd->flags.set_conn_params = true;
				}
            	break;

            case ID_SET_BLE_PHY_PARAMS:
            	if (	(p_vsd->params.preferred_gap_params.phys_params.tx_phys != p_vsd->incoming_uart_message.data[0]) ||
            			(p_vsd->params.preferred_gap_params.phys_params.rx_phys != p_vsd->incoming_uart_message.data[0]))
            	{
            		p_vsd->params.preferred_gap_params.phys_params.tx_phys = p_vsd->incoming_uart_message.data[0];
            		p_vsd->params.preferred_gap_params.phys_params.rx_phys = p_vsd->incoming_uart_message.data[0];
					p_vsd->flags.set_phy_params = true;
            	}
            	break;

            case ID_SET_BLE_ATT_SIZE_PARAMS:
            	if (	(p_vsd->params.preferred_gap_params.mtu_size_params.max_tx_octets != p_vsd->incoming_uart_message.data[0]) ||
						(p_vsd->params.preferred_gap_params.mtu_size_params.max_rx_octets != p_vsd->incoming_uart_message.data[1]))
				{
            		p_vsd->params.preferred_gap_params.mtu_size_params.max_tx_octets = p_vsd->incoming_uart_message.data[0];
            		p_vsd->params.preferred_gap_params.mtu_size_params.max_rx_octets = p_vsd->incoming_uart_message.data[1];
					p_vsd->flags.set_att_size_params = true;
				}
            	break;

            case ID_CHAR_BUFFER:
            	p_vsd->flags.notification_buffer = true;
            	memcpy(p_vsd->characteristic.buffer.data, p_vsd->incoming_uart_message.data, p_vsd->incoming_uart_message.length);
            	p_vsd->characteristic.buffer.length = p_vsd->incoming_uart_message.length;
            	break;

            case ID_SOFTWARE_RESET:
            	if ((p_vsd->incoming_uart_message.length == 1) && ((p_vsd->incoming_uart_message.data[0] == RESET_ALL) || (p_vsd->incoming_uart_message.data[0] == RESET_BLE_PICKIT)))
				{
					p_vsd->flags.exec_reset = true;
				}
                break;

            default:
                break;

        }
    }

    if (p_vsd->flags.w > 0)
    {

    	/** Send Serial message over UART */
    	if (p_vsd->flags.exec_reset)
		{
			if (!p_vsd->uart.transmit_in_progress)
			{
				sd_nvic_SystemReset();
			}
		}
    	else if (p_vsd->flags.boot_mode && is_vsd_send_request_free_for_id(ID_BOOT_MODE))
		{
			if (!vsd_send_request(_boot, false, ID_BOOT_MODE))
			{
				p_vsd->flags.boot_mode = false;
			}
		}
    	else if (p_vsd->flags.send_version && is_vsd_send_request_free_for_id(ID_GET_VERSION))
		{
			if (!vsd_send_request(_version, false, ID_GET_VERSION))
			{
				p_vsd->flags.send_version = false;
			}
		}
    	else if (p_vsd->flags.send_conn_status && is_vsd_send_request_free_for_id(ID_GET_CONN_STATUS))
		{
			if (!vsd_send_request(_conn_status, false, ID_GET_CONN_STATUS))
			{
				p_vsd->flags.send_conn_status = false;
			}
		}
    	else if (p_vsd->flags.send_ble_params && is_vsd_send_request_free_for_id(ID_GET_BLE_PARAMS))
		{
			if (!vsd_send_request(_ble_params, false, ID_GET_BLE_PARAMS))
			{
				p_vsd->flags.send_ble_params = false;
			}
		}
    	else if (p_vsd->flags.send_pa_lna_param && is_vsd_send_request_free_for_id(ID_PA_LNA))
		{
			if (!vsd_send_request(_pa_lna_param, false, ID_PA_LNA))
			{
				p_vsd->flags.send_pa_lna_param = false;
			}
		}
        else if (p_vsd->flags.transfer_ble_to_uart && is_vsd_send_request_free_for_id(ID_CHAR_BUFFER))
		{
        	if (!vsd_send_request(_transfer_ble_to_uart, false, ID_CHAR_BUFFER))
			{
				p_vsd->flags.transfer_ble_to_uart = false;
			}
		}
        else if (p_vsd->flags.extended_transfer_ble_to_uart && is_vsd_send_request_free_for_id(ID_CHAR_EXT_BUFFER_NO_CRC))
		{
        	if (!vsd_send_request(_extended_transfer_ble_to_uart, true, ID_CHAR_EXT_BUFFER_NO_CRC))
			{
				p_vsd->flags.extended_transfer_ble_to_uart = false;
			}
		}

    	/** Send NOTIFICATION over BLE */
    	if (p_vsd->flags.notification_buffer)
        {
        	if (!ble_pickit_app_notification_send(_notif_buffer))
        	{
        		p_vsd->flags.notification_buffer = false;
        	}
        }
    }

}

static void _boot(uint8_t *buffer)
{
	uint16_t crc = 0;

	buffer[0] = ID_BOOT_MODE;
	buffer[1] = 'N';
	buffer[2] = 1;
	buffer[3] = 0x23;
	crc = fu_crc_16_ibm(buffer, buffer[2]+3);
	buffer[buffer[2]+3] = (crc >> 8) & 0xff;
	buffer[buffer[2]+4] = (crc >> 0) & 0xff;
}

static void _version(uint8_t *buffer)
{
    uint8_t i = 0;
	uint16_t crc = 0;

	buffer[0] = ID_GET_VERSION;
	buffer[1] = 'N';
	buffer[2] = 7;
	for (i = 0 ; i < 7 ; i++)
	{
		buffer[3+i] = p_vsd->infos.vsd_version[i];
	}
	crc = fu_crc_16_ibm(buffer, buffer[2]+3);
	buffer[buffer[2]+3] = (crc >> 8) & 0xff;
	buffer[buffer[2]+4] = (crc >> 0) & 0xff;
}

static void _conn_status(uint8_t *buffer)
{
	uint16_t crc = 0;

	buffer[0] = ID_GET_CONN_STATUS;
	buffer[1] = 'N';
	buffer[2] = 1;
	buffer[3] = (p_vsd->status.is_connected_to_a_central ? 2 : 0) | (p_vsd->status.is_in_advertising_mode ? 1 : 0);
	crc = fu_crc_16_ibm(buffer, buffer[2]+3);
	buffer[buffer[2]+3] = (crc >> 8) & 0xff;
	buffer[buffer[2]+4] = (crc >> 0) & 0xff;
}

static void _ble_params(uint8_t *buffer)
{
	uint16_t crc = 0;

	buffer[0] = ID_GET_BLE_PARAMS;
	buffer[1] = 'N';
	buffer[2] = 13;
	buffer[3] = (p_vsd->params.current_gap_params.conn_params.min_conn_interval >> 8) & 0xff;
	buffer[4] = (p_vsd->params.current_gap_params.conn_params.min_conn_interval >> 0) & 0xff;
	buffer[5] = (p_vsd->params.current_gap_params.conn_params.max_conn_interval >> 8) & 0xff;
	buffer[6] = (p_vsd->params.current_gap_params.conn_params.max_conn_interval >> 0) & 0xff;
	buffer[7] = (p_vsd->params.current_gap_params.conn_params.slave_latency >> 8) & 0xff;
	buffer[8] = (p_vsd->params.current_gap_params.conn_params.slave_latency >> 0) & 0xff;
	buffer[9] = (p_vsd->params.current_gap_params.conn_params.conn_sup_timeout >> 8) & 0xff;
	buffer[10] = (p_vsd->params.current_gap_params.conn_params.conn_sup_timeout >> 0) & 0xff;
	buffer[11] = p_vsd->params.current_gap_params.phys_params.tx_phys;
	buffer[12] = p_vsd->params.current_gap_params.mtu_size_params.max_tx_octets;
	buffer[13] = p_vsd->params.current_gap_params.mtu_size_params.max_rx_octets;
	buffer[14] = p_vsd->params.pa_lna_enable;
	buffer[15] = p_vsd->params.leds_status_enable;
	crc = fu_crc_16_ibm(buffer, buffer[2]+3);
	buffer[buffer[2]+3] = (crc >> 8) & 0xff;
	buffer[buffer[2]+4] = (crc >> 0) & 0xff;
}

static void _pa_lna_param(uint8_t *buffer)
{
	uint16_t crc = 0;

	buffer[0] = ID_PA_LNA;
	buffer[1] = 'N';
	buffer[2] = 1;
	buffer[3] = p_vsd->params.pa_lna_enable;
	crc = fu_crc_16_ibm(buffer, buffer[2]+3);
	buffer[buffer[2]+3] = (crc >> 8) & 0xff;
	buffer[buffer[2]+4] = (crc >> 0) & 0xff;
}

static void _transfer_ble_to_uart(uint8_t *buffer)
{
    uint8_t i = 0;
	uint16_t crc = 0;

	buffer[0] = p_vsd->outgoing_uart_message.id;
	buffer[1] = p_vsd->outgoing_uart_message.type;
	buffer[2] = p_vsd->outgoing_uart_message.length;
	for (i = 0 ; i < buffer[2] ; i++)
	{
		buffer[3+i] = p_vsd->outgoing_uart_message.data[i];
	}
	crc = fu_crc_16_ibm(buffer, buffer[2]+3);
	buffer[buffer[2]+3] = (crc >> 8) & 0xff;
	buffer[buffer[2]+4] = (crc >> 0) & 0xff;
}

static void _extended_transfer_ble_to_uart(uint8_t *buffer)
{
	// No CRC for extended message
    memcpy(buffer, &p_vsd->outgoing_uart_extended_message, p_vsd->outgoing_uart_extended_message.length + 4);
}

static void _notif_buffer(uint8_t *buffer)
{
	uint8_t i = 0;

	buffer[0] = ID_CHAR_BUFFER;
	buffer[1] = p_vsd->characteristic.buffer.length;
	for (i = 0 ; i < buffer[1] ; i++)
	{
		buffer[2+i] = p_vsd->characteristic.buffer.data[i];
	}
}

static bool is_vsd_send_request_free_for_id(uint8_t id)
{
	return (current_id_requested == id) || (current_id_requested == ID_NONE);
}

static uint8_t vsd_send_request(p_function ptr, bool is_extended_message, uint8_t id)
{
    static state_machine_t sm;
	static uint8_t buffer[MAXIMUM_SIZE_EXTENDED_MESSAGE + 4] = {0};

	switch (sm.index)
	{
		case 0:
			sm.index++;
			sm.tick = mGetTick();
			current_id_requested = id;
			/* no break */
        case 1:

            if (!p_vsd->uart.transmit_in_progress && !p_vsd->uart.receive_in_progress)
            {
            	memset(buffer, 0, sizeof(buffer));
                sm.index++;
                sm.tick = mGetTick();
            }
            break;

        case 2:
        	if (mTickCompare(sm.tick) >= TICK_400US)
        	{
        		if (!p_vsd->uart.transmit_in_progress && !p_vsd->uart.receive_in_progress)
				{
					sm.index++;
					sm.tick = mGetTick();

					(*ptr)(buffer);
				}
        		else
        		{
        			sm.index = 1;
        		}
        	}
        	break;

		case 3:

			if (is_extended_message)
			{
				uint16_t data_length = (buffer[2] << 0) | (buffer[3] << 8);
				for (uint16_t i = 0 ; i <= (data_length + 4) ; i++)
				{
					do {} while (app_uart_put(buffer[i]) != NRF_SUCCESS);
				}
			}
			else
			{
				for (uint8_t i = 0 ; i <= (buffer[2] + 4) ; i++)
				{
					do {} while (app_uart_put(buffer[i]) != NRF_SUCCESS);
				}
			}

            p_vsd->uart.transmit_in_progress = true;

			sm.index++;
			sm.tick = mGetTick();
			break;

		case 4:

            if (!p_vsd->uart.transmit_in_progress)
            {
            	if (is_extended_message)
            	{
            		sm.index = 0;
            		current_id_requested = ID_NONE;
            	}
            	else
            	{
            		sm.index++;
					sm.tick = mGetTick();
            	}
            }
			break;

		case 5:

            if (p_vsd->uart.message_type == UART_ACK_MESSAGE)
            {
                p_vsd->uart.message_type = UART_NO_MESSAGE;
                sm.index = 0;
                current_id_requested = ID_NONE;
            }
            else if (p_vsd->uart.message_type == UART_NACK_MESSAGE)
            {
                p_vsd->uart.message_type = UART_NO_MESSAGE;
                sm.index = 3;
            }
            else if (mTickCompare(sm.tick) >= TICK_10MS)
            {
                sm.index = 3;
            }
			break;

		default:
            sm.index = 0;
            current_id_requested = ID_NONE;
			break;

	}

	return sm.index;
}

