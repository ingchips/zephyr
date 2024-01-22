att_dispatch_client_can_send_now = 0x000056a5;
att_dispatch_client_request_can_send_now_event = 0x000056ab;
att_dispatch_register_client = 0x000056b1;
att_dispatch_register_server = 0x000056c5;
att_dispatch_server_can_send_now = 0x000056d9;
att_dispatch_server_request_can_send_now_event = 0x000056df;
att_emit_general_event = 0x00005791;
att_server_can_send_packet_now = 0x00005ec5;
att_server_deferred_read_response = 0x00005ec9;
att_server_get_mtu = 0x00005ee1;
att_server_indicate = 0x00005f59;
att_server_init = 0x00005fdd;
att_server_notify = 0x00006019;
att_server_register_packet_handler = 0x00006131;
att_server_request_can_send_now_event = 0x0000613d;
att_set_db = 0x00006159;
att_set_read_callback = 0x0000616d;
att_set_write_callback = 0x00006179;
bd_addr_cmp = 0x000062e9;
bd_addr_copy = 0x000062ef;
bd_addr_to_str = 0x000062f9;
big_endian_read_16 = 0x00006331;
big_endian_read_32 = 0x00006339;
big_endian_store_16 = 0x0000634d;
big_endian_store_32 = 0x00006359;
btstack_config = 0x00006491;
btstack_memory_pool_create = 0x000065e1;
btstack_memory_pool_free = 0x00006609;
btstack_memory_pool_get = 0x00006669;
btstack_push_user_msg = 0x000066b1;
btstack_push_user_runnable = 0x000066bd;
btstack_reset = 0x000066c9;
char_for_nibble = 0x000069c3;
gap_add_dev_to_periodic_list = 0x00007269;
gap_add_whitelist = 0x00007279;
gap_aes_encrypt = 0x00007285;
gap_clear_white_lists = 0x000072bd;
gap_clr_adv_set = 0x000072c9;
gap_clr_periodic_adv_list = 0x000072d5;
gap_create_connection_cancel = 0x000072e1;
gap_disconnect = 0x000072ed;
gap_disconnect_all = 0x00007319;
gap_ext_create_connection = 0x00007359;
gap_get_connection_parameter_range = 0x0000744d;
gap_le_read_channel_map = 0x00007489;
gap_periodic_adv_create_sync = 0x000074f5;
gap_periodic_adv_create_sync_cancel = 0x00007519;
gap_periodic_adv_term_sync = 0x00007525;
gap_read_local_tx_power_level = 0x000075b9;
gap_read_periodic_adv_list_size = 0x000075c9;
gap_read_phy = 0x000075d5;
gap_read_remote_tx_power_level = 0x000075e1;
gap_read_remote_used_features = 0x000075f1;
gap_read_remote_version = 0x000075fd;
gap_read_rssi = 0x00007609;
gap_remove_whitelist = 0x00007615;
gap_rmv_adv_set = 0x00007691;
gap_rmv_dev_from_periodic_list = 0x0000769d;
gap_rx_test_v2 = 0x000076ad;
gap_set_adv_set_random_addr = 0x000076e5;
gap_set_callback_for_next_hci = 0x00007721;
gap_set_connection_parameter_range = 0x00007741;
gap_set_data_length = 0x00007759;
gap_set_def_phy = 0x00007771;
gap_set_default_subrate = 0x00007781;
gap_set_ext_adv_data = 0x0000779d;
gap_set_ext_adv_enable = 0x000077b5;
gap_set_ext_adv_para = 0x00007825;
gap_set_ext_scan_enable = 0x000078fd;
gap_set_ext_scan_para = 0x00007915;
gap_set_ext_scan_response_data = 0x000079b5;
gap_set_host_channel_classification = 0x000079cd;
gap_set_path_loss_reporting_enable = 0x000079dd;
gap_set_path_loss_reporting_param = 0x000079ed;
gap_set_periodic_adv_data = 0x00007a09;
gap_set_periodic_adv_enable = 0x00007a79;
gap_set_periodic_adv_para = 0x00007a89;
gap_set_phy = 0x00007aa1;
gap_set_random_device_address = 0x00007abd;
gap_set_tx_power_reporting_enable = 0x00007add;
gap_start_ccm = 0x00007afd;
gap_subrate_request = 0x00007b45;
gap_test_end = 0x00007b61;
gap_tx_test_v2 = 0x00007b6d;
gap_tx_test_v4 = 0x00007b85;
gap_update_connection_parameters = 0x00007ba9;
gap_vendor_tx_continuous_wave = 0x00007bed;
gatt_client_cancel_write = 0x00008115;
gatt_client_discover_characteristic_descriptors = 0x0000813b;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x0000817b;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x000081cb;
gatt_client_discover_characteristics_for_service = 0x0000821b;
gatt_client_discover_primary_services = 0x00008251;
gatt_client_discover_primary_services_by_uuid128 = 0x00008283;
gatt_client_discover_primary_services_by_uuid16 = 0x000082c7;
gatt_client_execute_write = 0x00008303;
gatt_client_find_included_services_for_service = 0x00008329;
gatt_client_get_mtu = 0x00008357;
gatt_client_is_ready = 0x000083f9;
gatt_client_listen_for_characteristic_value_updates = 0x0000840f;
gatt_client_prepare_write = 0x00008431;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x0000846d;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x00008497;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x0000849d;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x000084cb;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x000084d1;
gatt_client_read_multiple_characteristic_values = 0x000084ff;
gatt_client_read_value_of_characteristic_using_value_handle = 0x0000852f;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x0000855d;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x000085a9;
gatt_client_register_handler = 0x000085f5;
gatt_client_reliable_write_long_value_of_characteristic = 0x00008601;
gatt_client_signed_write_without_response = 0x00008a31;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00008af5;
gatt_client_write_client_characteristic_configuration = 0x00008b2f;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x00008b81;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008b91;
gatt_client_write_long_value_of_characteristic = 0x00008bcd;
gatt_client_write_long_value_of_characteristic_with_offset = 0x00008bdd;
gatt_client_write_value_of_characteristic = 0x00008c19;
gatt_client_write_value_of_characteristic_without_response = 0x00008c4f;
hci_add_event_handler = 0x0000a1e5;
hci_power_control = 0x0000a995;
hci_register_acl_packet_handler = 0x0000ab49;
kv_commit = 0x0000b2e5;
kv_get = 0x0000b341;
kv_init = 0x0000b34d;
kv_init_backend = 0x0000b3cd;
kv_put = 0x0000b3e1;
kv_remove = 0x0000b3ed;
kv_remove_all = 0x0000b421;
kv_value_modified = 0x0000b451;
kv_value_modified_of_key = 0x0000b46d;
kv_visit = 0x0000b479;
l2cap_add_event_handler = 0x0000b509;
l2cap_can_send_packet_now = 0x0000b519;
l2cap_create_le_credit_based_connection_request = 0x0000b6d5;
l2cap_credit_based_send = 0x0000b819;
l2cap_credit_based_send_continue = 0x0000b845;
l2cap_disconnect = 0x0000b8c1;
l2cap_get_le_credit_based_connection_credits = 0x0000bb11;
l2cap_get_peer_mtu_for_local_cid = 0x0000bb2d;
l2cap_init = 0x0000bf01;
l2cap_le_send_flow_control_credit = 0x0000bff7;
l2cap_max_le_mtu = 0x0000c301;
l2cap_register_packet_handler = 0x0000c429;
l2cap_register_service = 0x0000c435;
l2cap_request_can_send_now_event = 0x0000c545;
l2cap_request_connection_parameter_update = 0x0000c55f;
l2cap_send_echo_request = 0x0000ca39;
l2cap_unregister_service = 0x0000caf9;
le_device_db_add = 0x0000cb51;
le_device_db_find = 0x0000cc29;
le_device_db_from_key = 0x0000cc55;
le_device_db_iter_cur = 0x0000cc5d;
le_device_db_iter_cur_key = 0x0000cc61;
le_device_db_iter_init = 0x0000cc65;
le_device_db_iter_next = 0x0000cc6d;
le_device_db_remove_key = 0x0000cc93;
ll_ackable_packet_alloc = 0x0000ccbf;
ll_ackable_packet_get_status = 0x0000cdf9;
ll_ackable_packet_run = 0x0000ce69;
ll_ackable_packet_set_tx_data = 0x0000cf0d;
ll_adjust_conn_peer_tx_power = 0x0000cf35;
ll_aes_encrypt = 0x0000cf61;
ll_channel_monitor_alloc = 0x0000cfdd;
ll_channel_monitor_check_each_pdu = 0x0000d05f;
ll_channel_monitor_run = 0x0000d0c5;
ll_config = 0x0000d179;
ll_conn_abort = 0x0000d195;
ll_create_conn = 0x0000d1c9;
ll_dhkey_generated = 0x0000d445;
ll_free = 0x0000d479;
ll_get_conn_events_info = 0x0000d485;
ll_get_conn_info = 0x0000d569;
ll_get_heap_free_size = 0x0000d5b5;
ll_hint_on_ce_len = 0x0000d5c9;
ll_install_ecc_engine = 0x0000d601;
ll_legacy_adv_set_interval = 0x0000d60d;
ll_lock_frequency = 0x0000d61d;
ll_malloc = 0x0000d681;
ll_p256_key_pair_generated = 0x0000d689;
ll_raw_packet_alloc = 0x0000d83d;
ll_raw_packet_free = 0x0000d911;
ll_raw_packet_get_bare_rx_data = 0x0000d949;
ll_raw_packet_get_rx_data = 0x0000da0f;
ll_raw_packet_recv = 0x0000dab1;
ll_raw_packet_send = 0x0000db6d;
ll_raw_packet_set_bare_data = 0x0000dc55;
ll_raw_packet_set_bare_mode = 0x0000dc93;
ll_raw_packet_set_param = 0x0000dd99;
ll_raw_packet_set_tx_data = 0x0000ddf7;
ll_register_hci_acl_previewer = 0x0000de5d;
ll_scan_set_fixed_channel = 0x0000dec1;
ll_set_adv_access_address = 0x0000e0d9;
ll_set_adv_coded_scheme = 0x0000e0e5;
ll_set_conn_acl_report_latency = 0x0000e115;
ll_set_conn_coded_scheme = 0x0000e145;
ll_set_conn_interval_unit = 0x0000e171;
ll_set_conn_latency = 0x0000e17d;
ll_set_conn_tx_power = 0x0000e1ad;
ll_set_def_antenna = 0x0000e1e9;
ll_set_initiating_coded_scheme = 0x0000e205;
ll_set_max_conn_number = 0x0000e211;
ll_set_tx_power_range = 0x0000e2a5;
ll_unlock_frequency = 0x0000e2cd;
nibble_for_char = 0x0001fd5d;
platform_calibrate_rt_clk = 0x0001fdf9;
platform_call_on_stack = 0x000040ef;
platform_cancel_us_timer = 0x0001fdfd;
platform_config = 0x0001fe11;
platform_controller_run = 0x0001ff35;
platform_create_us_timer = 0x0001ff69;
platform_delete_timer = 0x0001ff7d;
platform_enable_irq = 0x0001ff85;
platform_get_gen_os_driver = 0x0001ffbd;
platform_get_link_layer_interf = 0x0001ffc9;
platform_get_task_handle = 0x0001ffd1;
platform_get_timer_counter = 0x0001ffe9;
platform_get_us_time = 0x0001ffed;
platform_get_version = 0x0001fff1;
platform_hrng = 0x0001fff9;
platform_init_controller = 0x00020001;
platform_os_idle_resumed_hook = 0x0002001d;
platform_patch_rf_init_data = 0x00020021;
platform_post_sleep_processing = 0x0002002d;
platform_pre_sleep_processing = 0x00020033;
platform_pre_suppress_ticks_and_sleep_processing = 0x00020039;
platform_printf = 0x0002003d;
platform_raise_assertion = 0x00020051;
platform_rand = 0x00020065;
platform_read_info = 0x00020069;
platform_read_persistent_reg = 0x00020099;
platform_reset = 0x000200a9;
platform_rt_rc_auto_tune = 0x000200cd;
platform_rt_rc_auto_tune2 = 0x000200d5;
platform_rt_rc_tune = 0x0002015d;
platform_set_abs_timer = 0x00020171;
platform_set_evt_callback = 0x00020175;
platform_set_evt_callback_table = 0x00020189;
platform_set_irq_callback = 0x00020195;
platform_set_irq_callback_table = 0x000201b1;
platform_set_rf_clk_source = 0x000201bd;
platform_set_rf_init_data = 0x000201c9;
platform_set_rf_power_mapping = 0x000201d5;
platform_set_timer = 0x000201e1;
platform_shutdown = 0x000201e5;
platform_switch_app = 0x000201e9;
platform_trace_raw = 0x00020215;
platform_write_persistent_reg = 0x0002022d;
printf_hexdump = 0x0002023d;
reverse_128 = 0x0002057d;
reverse_24 = 0x00020583;
reverse_256 = 0x00020589;
reverse_48 = 0x0002058f;
reverse_56 = 0x00020595;
reverse_64 = 0x0002059b;
reverse_bd_addr = 0x000205a1;
reverse_bytes = 0x000205a7;
sm_add_event_handler = 0x00020795;
sm_address_resolution_lookup = 0x000208d9;
sm_authenticated = 0x00020d31;
sm_authorization_decline = 0x00020d3f;
sm_authorization_grant = 0x00020d5f;
sm_authorization_state = 0x00020d7f;
sm_bonding_decline = 0x00020d99;
sm_config = 0x00021219;
sm_config_conn = 0x0002124d;
sm_encryption_key_size = 0x00021407;
sm_just_works_confirm = 0x0002214d;
sm_le_device_key = 0x0002253d;
sm_numeric_comparison_confirm = 0x00022693;
sm_passkey_input = 0x000226d1;
sm_private_random_address_generation_get = 0x00022cd9;
sm_private_random_address_generation_get_mode = 0x00022ce1;
sm_private_random_address_generation_set_mode = 0x00022ced;
sm_private_random_address_generation_set_update_period = 0x00022d15;
sm_register_external_ltk_callback = 0x00022fa1;
sm_register_oob_data_callback = 0x00022fad;
sm_register_sc_oob_data_callback = 0x00022fb9;
sm_request_pairing = 0x00022fc5;
sm_sc_generate_oob_data = 0x00023c01;
sm_send_security_request = 0x00023d31;
sm_set_accepted_stk_generation_methods = 0x00023d59;
sm_set_authentication_requirements = 0x00023d65;
sm_set_encryption_key_size_range = 0x00023d75;
sscanf_bd_addr = 0x00024181;
sysSetPublicDeviceAddr = 0x00024535;
uuid128_to_str = 0x00024b71;
uuid_add_bluetooth_prefix = 0x00024bc9;
uuid_has_bluetooth_prefix = 0x00024be9;
