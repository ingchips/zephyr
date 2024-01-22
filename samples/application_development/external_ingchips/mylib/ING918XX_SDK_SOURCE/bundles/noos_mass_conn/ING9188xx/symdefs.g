att_dispatch_client_can_send_now = 0x00005781;
att_dispatch_client_request_can_send_now_event = 0x00005787;
att_dispatch_register_client = 0x0000578d;
att_dispatch_register_server = 0x000057a1;
att_dispatch_server_can_send_now = 0x000057b5;
att_dispatch_server_request_can_send_now_event = 0x000057bb;
att_emit_general_event = 0x0000586d;
att_server_can_send_packet_now = 0x00005f9d;
att_server_deferred_read_response = 0x00005fa1;
att_server_get_mtu = 0x00005fb9;
att_server_indicate = 0x00006031;
att_server_init = 0x000060b5;
att_server_notify = 0x000060f1;
att_server_register_packet_handler = 0x00006209;
att_server_request_can_send_now_event = 0x00006215;
att_set_db = 0x00006231;
att_set_read_callback = 0x00006245;
att_set_write_callback = 0x00006251;
bd_addr_cmp = 0x000063c1;
bd_addr_copy = 0x000063c7;
bd_addr_to_str = 0x000063d1;
big_endian_read_16 = 0x00006409;
big_endian_read_32 = 0x00006411;
big_endian_store_16 = 0x00006425;
big_endian_store_32 = 0x00006431;
btstack_config = 0x00006569;
btstack_memory_pool_create = 0x000066b7;
btstack_memory_pool_free = 0x000066e1;
btstack_memory_pool_get = 0x00006741;
btstack_push_user_msg = 0x00006789;
btstack_push_user_runnable = 0x00006795;
btstack_reset = 0x000067a1;
char_for_nibble = 0x00006a65;
gap_add_dev_to_periodic_list = 0x00007381;
gap_add_whitelist = 0x00007391;
gap_aes_encrypt = 0x0000739d;
gap_clear_white_lists = 0x000073d5;
gap_clr_adv_set = 0x000073e1;
gap_clr_periodic_adv_list = 0x000073ed;
gap_create_connection_cancel = 0x000073f9;
gap_default_periodic_adv_sync_transfer_param = 0x00007405;
gap_disconnect = 0x0000741d;
gap_disconnect_all = 0x00007449;
gap_ext_create_connection = 0x00007489;
gap_get_connection_parameter_range = 0x00007561;
gap_le_read_channel_map = 0x00007599;
gap_periodic_adv_create_sync = 0x00007605;
gap_periodic_adv_create_sync_cancel = 0x00007629;
gap_periodic_adv_set_info_transfer = 0x00007635;
gap_periodic_adv_sync_transfer = 0x00007645;
gap_periodic_adv_sync_transfer_param = 0x00007655;
gap_periodic_adv_term_sync = 0x00007671;
gap_read_antenna_info = 0x000076f9;
gap_read_periodic_adv_list_size = 0x00007705;
gap_read_phy = 0x00007711;
gap_read_remote_used_features = 0x0000771d;
gap_read_remote_version = 0x00007729;
gap_read_rssi = 0x00007735;
gap_remove_whitelist = 0x00007741;
gap_rmv_adv_set = 0x000077bd;
gap_rmv_dev_from_periodic_list = 0x000077c9;
gap_rx_test_v2 = 0x000077d9;
gap_rx_test_v3 = 0x000077e9;
gap_set_adv_set_random_addr = 0x00007835;
gap_set_callback_for_next_hci = 0x00007871;
gap_set_connection_cte_request_enable = 0x00007899;
gap_set_connection_cte_response_enable = 0x000078b5;
gap_set_connection_cte_rx_param = 0x000078c5;
gap_set_connection_cte_tx_param = 0x00007919;
gap_set_connection_parameter_range = 0x00007965;
gap_set_connectionless_cte_tx_enable = 0x00007981;
gap_set_connectionless_cte_tx_param = 0x00007991;
gap_set_connectionless_iq_sampling_enable = 0x000079ed;
gap_set_data_length = 0x00007a49;
gap_set_def_phy = 0x00007a61;
gap_set_ext_adv_data = 0x00007a71;
gap_set_ext_adv_enable = 0x00007a89;
gap_set_ext_adv_para = 0x00007af9;
gap_set_ext_scan_enable = 0x00007bd1;
gap_set_ext_scan_para = 0x00007be9;
gap_set_ext_scan_response_data = 0x00007c89;
gap_set_host_channel_classification = 0x00007ca1;
gap_set_periodic_adv_data = 0x00007cb1;
gap_set_periodic_adv_enable = 0x00007d21;
gap_set_periodic_adv_para = 0x00007d31;
gap_set_periodic_adv_rx_enable = 0x00007d49;
gap_set_phy = 0x00007d59;
gap_set_random_device_address = 0x00007d75;
gap_start_ccm = 0x00007da5;
gap_test_end = 0x00007ded;
gap_tx_test_v2 = 0x00007df9;
gap_tx_test_v4 = 0x00007e11;
gap_update_connection_parameters = 0x00007e35;
gap_vendor_tx_continuous_wave = 0x00007e79;
gatt_client_cancel_write = 0x000083a1;
gatt_client_discover_characteristic_descriptors = 0x000083c7;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x00008407;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x00008457;
gatt_client_discover_characteristics_for_service = 0x000084a7;
gatt_client_discover_primary_services = 0x000084dd;
gatt_client_discover_primary_services_by_uuid128 = 0x0000850f;
gatt_client_discover_primary_services_by_uuid16 = 0x00008553;
gatt_client_execute_write = 0x0000858f;
gatt_client_find_included_services_for_service = 0x000085b5;
gatt_client_get_mtu = 0x000085e3;
gatt_client_is_ready = 0x00008685;
gatt_client_listen_for_characteristic_value_updates = 0x0000869b;
gatt_client_prepare_write = 0x000086bd;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x000086f9;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x00008723;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008729;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x00008757;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x0000875d;
gatt_client_read_multiple_characteristic_values = 0x0000878b;
gatt_client_read_value_of_characteristic_using_value_handle = 0x000087bb;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x000087e9;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x00008835;
gatt_client_register_handler = 0x00008881;
gatt_client_reliable_write_long_value_of_characteristic = 0x0000888d;
gatt_client_signed_write_without_response = 0x00008cbd;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00008d81;
gatt_client_write_client_characteristic_configuration = 0x00008dbb;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x00008e0d;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008e1d;
gatt_client_write_long_value_of_characteristic = 0x00008e59;
gatt_client_write_long_value_of_characteristic_with_offset = 0x00008e69;
gatt_client_write_value_of_characteristic = 0x00008ea5;
gatt_client_write_value_of_characteristic_without_response = 0x00008edb;
hci_add_event_handler = 0x0000a41d;
hci_power_control = 0x0000abc1;
hci_register_acl_packet_handler = 0x0000ad75;
kv_commit = 0x0000b525;
kv_get = 0x0000b581;
kv_init = 0x0000b58d;
kv_init_backend = 0x0000b60d;
kv_put = 0x0000b621;
kv_remove = 0x0000b62d;
kv_remove_all = 0x0000b661;
kv_value_modified = 0x0000b691;
kv_value_modified_of_key = 0x0000b6ad;
kv_visit = 0x0000b6b9;
l2cap_add_event_handler = 0x0000b749;
l2cap_can_send_packet_now = 0x0000b759;
l2cap_create_le_credit_based_connection_request = 0x0000b915;
l2cap_credit_based_send = 0x0000ba59;
l2cap_credit_based_send_continue = 0x0000ba85;
l2cap_disconnect = 0x0000bb01;
l2cap_get_le_credit_based_connection_credits = 0x0000bd51;
l2cap_get_peer_mtu_for_local_cid = 0x0000bd6d;
l2cap_init = 0x0000c141;
l2cap_le_send_flow_control_credit = 0x0000c237;
l2cap_max_le_mtu = 0x0000c541;
l2cap_register_packet_handler = 0x0000c669;
l2cap_register_service = 0x0000c675;
l2cap_request_can_send_now_event = 0x0000c785;
l2cap_request_connection_parameter_update = 0x0000c79f;
l2cap_send_echo_request = 0x0000cc79;
l2cap_unregister_service = 0x0000cd39;
le_device_db_add = 0x0000cd91;
le_device_db_find = 0x0000ce69;
le_device_db_from_key = 0x0000ce95;
le_device_db_iter_cur = 0x0000ce9d;
le_device_db_iter_cur_key = 0x0000cea1;
le_device_db_iter_init = 0x0000cea5;
le_device_db_iter_next = 0x0000cead;
le_device_db_remove_key = 0x0000ced3;
ll_aes_encrypt = 0x0000cf01;
ll_config = 0x0000cf7d;
ll_free = 0x0000cfb3;
ll_get_heap_free_size = 0x0000cfbd;
ll_hint_on_ce_len = 0x0000cfd1;
ll_legacy_adv_set_interval = 0x0000d009;
ll_malloc = 0x0000d019;
ll_query_timing_info = 0x0000d151;
ll_register_hci_acl_previewer = 0x0000d19d;
ll_scan_set_fixed_channel = 0x0000d201;
ll_set_adv_access_address = 0x0000d419;
ll_set_adv_coded_scheme = 0x0000d425;
ll_set_conn_acl_report_latency = 0x0000d455;
ll_set_conn_coded_scheme = 0x0000d485;
ll_set_conn_latency = 0x0000d4b1;
ll_set_conn_tx_power = 0x0000d4e1;
ll_set_def_antenna = 0x0000d529;
ll_set_initiating_coded_scheme = 0x0000d545;
ll_set_max_conn_number = 0x0000d551;
nibble_for_char = 0x0001e5c5;
platform_32k_rc_auto_tune = 0x0001e661;
platform_32k_rc_tune = 0x0001e6dd;
platform_calibrate_32k = 0x0001e6f1;
platform_config = 0x0001e6f5;
platform_controller_run = 0x0001e819;
platform_delete_timer = 0x0001e84d;
platform_enable_irq = 0x0001e855;
platform_get_gen_os_driver = 0x0001e88d;
platform_get_link_layer_interf = 0x0001e899;
platform_get_task_handle = 0x0001e8a1;
platform_get_timer_counter = 0x0001e8b9;
platform_get_us_time = 0x0001e8bd;
platform_get_version = 0x0001e8c1;
platform_hrng = 0x0001e8c9;
platform_init_controller = 0x0001e8d1;
platform_os_idle_resumed_hook = 0x0001e8ed;
platform_patch_rf_init_data = 0x0001e8f1;
platform_post_sleep_processing = 0x0001e8fd;
platform_pre_sleep_processing = 0x0001e903;
platform_pre_suppress_ticks_and_sleep_processing = 0x0001e909;
platform_printf = 0x0001e90d;
platform_raise_assertion = 0x0001e921;
platform_rand = 0x0001e935;
platform_read_info = 0x0001e939;
platform_read_persistent_reg = 0x0001e969;
platform_reset = 0x0001e979;
platform_set_abs_timer = 0x0001e99d;
platform_set_evt_callback = 0x0001e9a1;
platform_set_evt_callback_table = 0x0001e9b5;
platform_set_irq_callback = 0x0001e9c1;
platform_set_irq_callback_table = 0x0001e9dd;
platform_set_rf_clk_source = 0x0001e9e9;
platform_set_rf_init_data = 0x0001e9f5;
platform_set_rf_power_mapping = 0x0001ea01;
platform_set_timer = 0x0001ea0d;
platform_shutdown = 0x0001ea11;
platform_switch_app = 0x0001ea15;
platform_trace_raw = 0x0001ea41;
platform_write_persistent_reg = 0x0001ea59;
printf_hexdump = 0x0001ea69;
reverse_128 = 0x0001ede1;
reverse_24 = 0x0001ede7;
reverse_256 = 0x0001eded;
reverse_48 = 0x0001edf3;
reverse_56 = 0x0001edf9;
reverse_64 = 0x0001edff;
reverse_bd_addr = 0x0001ee05;
reverse_bytes = 0x0001ee0b;
sm_add_event_handler = 0x0001f0f9;
sm_address_resolution_lookup = 0x0001f251;
sm_authenticated = 0x0001f5cd;
sm_authorization_decline = 0x0001f5db;
sm_authorization_grant = 0x0001f5fb;
sm_authorization_state = 0x0001f61b;
sm_bonding_decline = 0x0001f635;
sm_config = 0x0001fa91;
sm_config_conn = 0x0001faa9;
sm_encryption_key_size = 0x0001fc5f;
sm_just_works_confirm = 0x000201e5;
sm_le_device_key = 0x00020531;
sm_passkey_input = 0x000205c7;
sm_private_random_address_generation_get = 0x00020981;
sm_private_random_address_generation_get_mode = 0x00020989;
sm_private_random_address_generation_set_mode = 0x00020995;
sm_private_random_address_generation_set_update_period = 0x000209bd;
sm_register_external_ltk_callback = 0x00020af9;
sm_register_oob_data_callback = 0x00020b05;
sm_request_pairing = 0x00020b11;
sm_send_security_request = 0x000215e7;
sm_set_accepted_stk_generation_methods = 0x0002160d;
sm_set_authentication_requirements = 0x00021619;
sm_set_encryption_key_size_range = 0x00021625;
sscanf_bd_addr = 0x00021981;
sysSetPublicDeviceAddr = 0x00021d35;
uuid128_to_str = 0x000224d5;
uuid_add_bluetooth_prefix = 0x0002252d;
uuid_has_bluetooth_prefix = 0x0002254d;
