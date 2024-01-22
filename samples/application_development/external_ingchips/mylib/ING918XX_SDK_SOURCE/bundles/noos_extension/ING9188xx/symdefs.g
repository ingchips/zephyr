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
gap_add_dev_to_periodic_list = 0x0000737d;
gap_add_whitelist = 0x0000738d;
gap_aes_encrypt = 0x00007399;
gap_clear_white_lists = 0x000073d1;
gap_clr_adv_set = 0x000073dd;
gap_clr_periodic_adv_list = 0x000073e9;
gap_create_connection_cancel = 0x000073f5;
gap_default_periodic_adv_sync_transfer_param = 0x00007401;
gap_disconnect = 0x00007419;
gap_disconnect_all = 0x00007445;
gap_ext_create_connection = 0x00007485;
gap_get_connection_parameter_range = 0x0000755d;
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
gap_set_connection_cte_request_enable = 0x00007891;
gap_set_connection_cte_response_enable = 0x000078ad;
gap_set_connection_cte_rx_param = 0x000078bd;
gap_set_connection_cte_tx_param = 0x00007911;
gap_set_connection_parameter_range = 0x0000795d;
gap_set_connectionless_cte_tx_enable = 0x00007975;
gap_set_connectionless_cte_tx_param = 0x00007985;
gap_set_connectionless_iq_sampling_enable = 0x000079e1;
gap_set_data_length = 0x00007a3d;
gap_set_def_phy = 0x00007a55;
gap_set_ext_adv_data = 0x00007a65;
gap_set_ext_adv_enable = 0x00007a7d;
gap_set_ext_adv_para = 0x00007aed;
gap_set_ext_scan_enable = 0x00007bc5;
gap_set_ext_scan_para = 0x00007bdd;
gap_set_ext_scan_response_data = 0x00007c7d;
gap_set_host_channel_classification = 0x00007c95;
gap_set_periodic_adv_data = 0x00007ca5;
gap_set_periodic_adv_enable = 0x00007d15;
gap_set_periodic_adv_para = 0x00007d25;
gap_set_periodic_adv_rx_enable = 0x00007d3d;
gap_set_phy = 0x00007d4d;
gap_set_random_device_address = 0x00007d69;
gap_start_ccm = 0x00007d99;
gap_test_end = 0x00007de1;
gap_tx_test_v2 = 0x00007ded;
gap_tx_test_v4 = 0x00007e05;
gap_update_connection_parameters = 0x00007e29;
gap_vendor_tx_continuous_wave = 0x00007e6d;
gatt_client_cancel_write = 0x00008395;
gatt_client_discover_characteristic_descriptors = 0x000083bb;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x000083fb;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x0000844b;
gatt_client_discover_characteristics_for_service = 0x0000849b;
gatt_client_discover_primary_services = 0x000084d1;
gatt_client_discover_primary_services_by_uuid128 = 0x00008503;
gatt_client_discover_primary_services_by_uuid16 = 0x00008547;
gatt_client_execute_write = 0x00008583;
gatt_client_find_included_services_for_service = 0x000085a9;
gatt_client_get_mtu = 0x000085d7;
gatt_client_is_ready = 0x00008679;
gatt_client_listen_for_characteristic_value_updates = 0x0000868f;
gatt_client_prepare_write = 0x000086b1;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x000086ed;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x00008717;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x0000871d;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x0000874b;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x00008751;
gatt_client_read_multiple_characteristic_values = 0x0000877f;
gatt_client_read_value_of_characteristic_using_value_handle = 0x000087af;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x000087dd;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x00008829;
gatt_client_register_handler = 0x00008875;
gatt_client_reliable_write_long_value_of_characteristic = 0x00008881;
gatt_client_signed_write_without_response = 0x00008cb1;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00008d75;
gatt_client_write_client_characteristic_configuration = 0x00008daf;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x00008e01;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008e11;
gatt_client_write_long_value_of_characteristic = 0x00008e4d;
gatt_client_write_long_value_of_characteristic_with_offset = 0x00008e5d;
gatt_client_write_value_of_characteristic = 0x00008e99;
gatt_client_write_value_of_characteristic_without_response = 0x00008ecf;
hci_add_event_handler = 0x0000a411;
hci_power_control = 0x0000abb1;
hci_register_acl_packet_handler = 0x0000ad65;
kv_commit = 0x0000b501;
kv_get = 0x0000b55d;
kv_init = 0x0000b569;
kv_init_backend = 0x0000b5e9;
kv_put = 0x0000b5fd;
kv_remove = 0x0000b609;
kv_remove_all = 0x0000b63d;
kv_value_modified = 0x0000b66d;
kv_value_modified_of_key = 0x0000b689;
kv_visit = 0x0000b695;
l2cap_add_event_handler = 0x0000b725;
l2cap_can_send_packet_now = 0x0000b735;
l2cap_create_le_credit_based_connection_request = 0x0000b8f1;
l2cap_credit_based_send = 0x0000ba35;
l2cap_credit_based_send_continue = 0x0000ba61;
l2cap_disconnect = 0x0000badd;
l2cap_get_le_credit_based_connection_credits = 0x0000bd2d;
l2cap_get_peer_mtu_for_local_cid = 0x0000bd49;
l2cap_init = 0x0000c11d;
l2cap_le_send_flow_control_credit = 0x0000c213;
l2cap_max_le_mtu = 0x0000c51d;
l2cap_register_packet_handler = 0x0000c645;
l2cap_register_service = 0x0000c651;
l2cap_request_can_send_now_event = 0x0000c761;
l2cap_request_connection_parameter_update = 0x0000c77b;
l2cap_send_echo_request = 0x0000cc55;
l2cap_unregister_service = 0x0000cd15;
le_device_db_add = 0x0000cd6d;
le_device_db_find = 0x0000ce45;
le_device_db_from_key = 0x0000ce71;
le_device_db_iter_cur = 0x0000ce79;
le_device_db_iter_cur_key = 0x0000ce7d;
le_device_db_iter_init = 0x0000ce81;
le_device_db_iter_next = 0x0000ce89;
le_device_db_remove_key = 0x0000ceaf;
ll_ackable_packet_alloc = 0x0000cedb;
ll_ackable_packet_get_status = 0x0000d00d;
ll_ackable_packet_run = 0x0000d07d;
ll_ackable_packet_set_tx_data = 0x0000d119;
ll_aes_encrypt = 0x0000d135;
ll_attach_cte_to_adv_set = 0x0000d1b1;
ll_channel_monitor_alloc = 0x0000d349;
ll_channel_monitor_check_each_pdu = 0x0000d3cb;
ll_channel_monitor_run = 0x0000d431;
ll_config = 0x0000d4e5;
ll_free = 0x0000d51b;
ll_get_heap_free_size = 0x0000d525;
ll_hint_on_ce_len = 0x0000d539;
ll_legacy_adv_set_interval = 0x0000d571;
ll_lock_frequency = 0x0000d581;
ll_malloc = 0x0000d5e5;
ll_query_timing_info = 0x0000d71d;
ll_raw_packet_alloc = 0x0000d769;
ll_raw_packet_free = 0x0000d83d;
ll_raw_packet_get_bare_rx_data = 0x0000d875;
ll_raw_packet_get_iq_samples = 0x0000d93b;
ll_raw_packet_get_rx_data = 0x0000d9d5;
ll_raw_packet_recv = 0x0000da75;
ll_raw_packet_send = 0x0000db31;
ll_raw_packet_set_bare_data = 0x0000dc19;
ll_raw_packet_set_bare_mode = 0x0000dc57;
ll_raw_packet_set_fake_cte_info = 0x0000dd5d;
ll_raw_packet_set_param = 0x0000dd7f;
ll_raw_packet_set_rx_cte = 0x0000dddd;
ll_raw_packet_set_tx_cte = 0x0000de73;
ll_raw_packet_set_tx_data = 0x0000deb1;
ll_register_hci_acl_previewer = 0x0000df15;
ll_scan_set_fixed_channel = 0x0000df79;
ll_scanner_enable_iq_sampling = 0x0000df85;
ll_set_adv_access_address = 0x0000e239;
ll_set_adv_coded_scheme = 0x0000e245;
ll_set_conn_acl_report_latency = 0x0000e275;
ll_set_conn_coded_scheme = 0x0000e2a5;
ll_set_conn_interval_unit = 0x0000e2d1;
ll_set_conn_latency = 0x0000e2dd;
ll_set_conn_tx_power = 0x0000e30d;
ll_set_def_antenna = 0x0000e355;
ll_set_initiating_coded_scheme = 0x0000e371;
ll_set_max_conn_number = 0x0000e37d;
ll_unlock_frequency = 0x0000e411;
nibble_for_char = 0x0001f695;
platform_32k_rc_auto_tune = 0x0001f731;
platform_32k_rc_tune = 0x0001f7ad;
platform_calibrate_32k = 0x0001f7c1;
platform_config = 0x0001f7c5;
platform_controller_run = 0x0001f8e9;
platform_delete_timer = 0x0001f91d;
platform_enable_irq = 0x0001f925;
platform_get_gen_os_driver = 0x0001f95d;
platform_get_link_layer_interf = 0x0001f969;
platform_get_task_handle = 0x0001f971;
platform_get_timer_counter = 0x0001f989;
platform_get_us_time = 0x0001f98d;
platform_get_version = 0x0001f991;
platform_hrng = 0x0001f999;
platform_init_controller = 0x0001f9a1;
platform_os_idle_resumed_hook = 0x0001f9bd;
platform_patch_rf_init_data = 0x0001f9c1;
platform_post_sleep_processing = 0x0001f9cd;
platform_pre_sleep_processing = 0x0001f9d3;
platform_pre_suppress_ticks_and_sleep_processing = 0x0001f9d9;
platform_printf = 0x0001f9dd;
platform_raise_assertion = 0x0001f9f1;
platform_rand = 0x0001fa05;
platform_read_info = 0x0001fa09;
platform_read_persistent_reg = 0x0001fa39;
platform_reset = 0x0001fa49;
platform_set_abs_timer = 0x0001fa6d;
platform_set_evt_callback = 0x0001fa71;
platform_set_evt_callback_table = 0x0001fa85;
platform_set_irq_callback = 0x0001fa91;
platform_set_irq_callback_table = 0x0001faad;
platform_set_rf_clk_source = 0x0001fab9;
platform_set_rf_init_data = 0x0001fac5;
platform_set_rf_power_mapping = 0x0001fad1;
platform_set_timer = 0x0001fadd;
platform_shutdown = 0x0001fae1;
platform_switch_app = 0x0001fae5;
platform_trace_raw = 0x0001fb11;
platform_write_persistent_reg = 0x0001fb29;
printf_hexdump = 0x0001fb39;
reverse_128 = 0x0001feb1;
reverse_24 = 0x0001feb7;
reverse_256 = 0x0001febd;
reverse_48 = 0x0001fec3;
reverse_56 = 0x0001fec9;
reverse_64 = 0x0001fecf;
reverse_bd_addr = 0x0001fed5;
reverse_bytes = 0x0001fedb;
sm_add_event_handler = 0x000201f9;
sm_address_resolution_lookup = 0x00020351;
sm_authenticated = 0x000206cd;
sm_authorization_decline = 0x000206db;
sm_authorization_grant = 0x000206fb;
sm_authorization_state = 0x0002071b;
sm_bonding_decline = 0x00020735;
sm_config = 0x00020b91;
sm_config_conn = 0x00020ba9;
sm_encryption_key_size = 0x00020d5f;
sm_just_works_confirm = 0x000212e5;
sm_le_device_key = 0x00021631;
sm_passkey_input = 0x000216c7;
sm_private_random_address_generation_get = 0x00021a81;
sm_private_random_address_generation_get_mode = 0x00021a89;
sm_private_random_address_generation_set_mode = 0x00021a95;
sm_private_random_address_generation_set_update_period = 0x00021abd;
sm_register_external_ltk_callback = 0x00021bf9;
sm_register_oob_data_callback = 0x00021c05;
sm_request_pairing = 0x00021c11;
sm_send_security_request = 0x000226e7;
sm_set_accepted_stk_generation_methods = 0x0002270d;
sm_set_authentication_requirements = 0x00022719;
sm_set_encryption_key_size_range = 0x00022725;
sscanf_bd_addr = 0x00022af5;
sysSetPublicDeviceAddr = 0x00022ea9;
uuid128_to_str = 0x00023649;
uuid_add_bluetooth_prefix = 0x000236a1;
uuid_has_bluetooth_prefix = 0x000236c1;
