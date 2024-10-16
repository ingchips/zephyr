att_dispatch_client_can_send_now = 0x000058bd;
att_dispatch_client_request_can_send_now_event = 0x000058c3;
att_dispatch_register_client = 0x000058c9;
att_dispatch_register_server = 0x000058dd;
att_dispatch_server_can_send_now = 0x000058f1;
att_dispatch_server_request_can_send_now_event = 0x000058f7;
att_emit_general_event = 0x000059a9;
att_server_can_send_packet_now = 0x000060dd;
att_server_deferred_read_response = 0x000060e1;
att_server_get_mtu = 0x000060f9;
att_server_indicate = 0x00006171;
att_server_init = 0x000061f5;
att_server_notify = 0x00006231;
att_server_register_packet_handler = 0x00006349;
att_server_request_can_send_now_event = 0x00006355;
att_set_db = 0x00006371;
att_set_read_callback = 0x00006385;
att_set_write_callback = 0x00006391;
bd_addr_cmp = 0x00006501;
bd_addr_copy = 0x00006507;
bd_addr_to_str = 0x00006511;
big_endian_read_16 = 0x00006549;
big_endian_read_32 = 0x00006551;
big_endian_store_16 = 0x00006565;
big_endian_store_32 = 0x00006571;
btstack_config = 0x000066c5;
btstack_memory_pool_create = 0x00006803;
btstack_memory_pool_free = 0x0000682d;
btstack_memory_pool_get = 0x0000688d;
btstack_push_user_msg = 0x000068f5;
btstack_push_user_runnable = 0x00006901;
btstack_reset = 0x0000690d;
char_for_nibble = 0x00006be9;
eTaskConfirmSleepModeStatus = 0x00006ed1;
gap_add_dev_to_periodic_list = 0x0000753d;
gap_add_whitelist = 0x0000754d;
gap_aes_encrypt = 0x00007559;
gap_clear_white_lists = 0x00007591;
gap_clr_adv_set = 0x0000759d;
gap_clr_periodic_adv_list = 0x000075a9;
gap_create_connection_cancel = 0x000075b5;
gap_default_periodic_adv_sync_transfer_param = 0x000075c1;
gap_disconnect = 0x000075d9;
gap_disconnect_all = 0x00007605;
gap_ext_create_connection = 0x00007645;
gap_get_connection_parameter_range = 0x0000771d;
gap_le_read_channel_map = 0x00007759;
gap_periodic_adv_create_sync = 0x000077c5;
gap_periodic_adv_create_sync_cancel = 0x000077e9;
gap_periodic_adv_set_info_transfer = 0x000077f5;
gap_periodic_adv_sync_transfer = 0x00007805;
gap_periodic_adv_sync_transfer_param = 0x00007815;
gap_periodic_adv_term_sync = 0x00007831;
gap_read_antenna_info = 0x000078b9;
gap_read_periodic_adv_list_size = 0x000078c5;
gap_read_phy = 0x000078d1;
gap_read_remote_used_features = 0x000078dd;
gap_read_remote_version = 0x000078e9;
gap_read_rssi = 0x000078f5;
gap_remove_whitelist = 0x00007901;
gap_rmv_adv_set = 0x0000797d;
gap_rmv_dev_from_periodic_list = 0x00007989;
gap_rx_test_v2 = 0x00007999;
gap_rx_test_v3 = 0x000079a9;
gap_set_adv_set_random_addr = 0x000079f5;
gap_set_callback_for_next_hci = 0x00007a31;
gap_set_connection_cte_request_enable = 0x00007a51;
gap_set_connection_cte_response_enable = 0x00007a6d;
gap_set_connection_cte_rx_param = 0x00007a7d;
gap_set_connection_cte_tx_param = 0x00007ad1;
gap_set_connection_parameter_range = 0x00007b1d;
gap_set_connectionless_cte_tx_enable = 0x00007b35;
gap_set_connectionless_cte_tx_param = 0x00007b45;
gap_set_connectionless_iq_sampling_enable = 0x00007ba1;
gap_set_data_length = 0x00007bfd;
gap_set_def_phy = 0x00007c15;
gap_set_ext_adv_data = 0x00007c25;
gap_set_ext_adv_enable = 0x00007c3d;
gap_set_ext_adv_para = 0x00007cad;
gap_set_ext_scan_enable = 0x00007d85;
gap_set_ext_scan_para = 0x00007d9d;
gap_set_ext_scan_response_data = 0x00007e3d;
gap_set_host_channel_classification = 0x00007e55;
gap_set_periodic_adv_data = 0x00007e65;
gap_set_periodic_adv_enable = 0x00007ed5;
gap_set_periodic_adv_para = 0x00007ee5;
gap_set_periodic_adv_rx_enable = 0x00007efd;
gap_set_phy = 0x00007f0d;
gap_set_random_device_address = 0x00007f29;
gap_start_ccm = 0x00007f59;
gap_test_end = 0x00007fa1;
gap_tx_test_v2 = 0x00007fad;
gap_tx_test_v4 = 0x00007fc5;
gap_update_connection_parameters = 0x00007fe9;
gap_vendor_tx_continuous_wave = 0x0000802d;
gatt_client_cancel_write = 0x00008555;
gatt_client_discover_characteristic_descriptors = 0x0000857b;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x000085bb;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x0000860b;
gatt_client_discover_characteristics_for_service = 0x0000865b;
gatt_client_discover_primary_services = 0x00008691;
gatt_client_discover_primary_services_by_uuid128 = 0x000086c3;
gatt_client_discover_primary_services_by_uuid16 = 0x00008707;
gatt_client_execute_write = 0x00008743;
gatt_client_find_included_services_for_service = 0x00008769;
gatt_client_get_mtu = 0x00008797;
gatt_client_is_ready = 0x00008839;
gatt_client_listen_for_characteristic_value_updates = 0x0000884f;
gatt_client_prepare_write = 0x00008871;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x000088ad;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x000088d7;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x000088dd;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x0000890b;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x00008911;
gatt_client_read_multiple_characteristic_values = 0x0000893f;
gatt_client_read_value_of_characteristic_using_value_handle = 0x0000896f;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x0000899d;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x000089e9;
gatt_client_register_handler = 0x00008a35;
gatt_client_reliable_write_long_value_of_characteristic = 0x00008a41;
gatt_client_signed_write_without_response = 0x00008e71;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00008f35;
gatt_client_write_client_characteristic_configuration = 0x00008f6f;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x00008fc1;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008fd1;
gatt_client_write_long_value_of_characteristic = 0x0000900d;
gatt_client_write_long_value_of_characteristic_with_offset = 0x0000901d;
gatt_client_write_value_of_characteristic = 0x00009059;
gatt_client_write_value_of_characteristic_without_response = 0x0000908f;
hci_add_event_handler = 0x0000a5d1;
hci_power_control = 0x0000ad71;
hci_register_acl_packet_handler = 0x0000af25;
kv_commit = 0x0000b699;
kv_get = 0x0000b6f5;
kv_init = 0x0000b701;
kv_init_backend = 0x0000b781;
kv_put = 0x0000b795;
kv_remove = 0x0000b7a1;
kv_remove_all = 0x0000b7d5;
kv_value_modified = 0x0000b805;
kv_value_modified_of_key = 0x0000b821;
kv_visit = 0x0000b82d;
l2cap_add_event_handler = 0x0000b8bd;
l2cap_can_send_packet_now = 0x0000b8cd;
l2cap_create_le_credit_based_connection_request = 0x0000ba89;
l2cap_credit_based_send = 0x0000bbcd;
l2cap_credit_based_send_continue = 0x0000bbf9;
l2cap_disconnect = 0x0000bc75;
l2cap_get_le_credit_based_connection_credits = 0x0000bec5;
l2cap_get_peer_mtu_for_local_cid = 0x0000bee1;
l2cap_init = 0x0000c2b5;
l2cap_le_send_flow_control_credit = 0x0000c3ab;
l2cap_max_le_mtu = 0x0000c6b5;
l2cap_register_packet_handler = 0x0000c7dd;
l2cap_register_service = 0x0000c7e9;
l2cap_request_can_send_now_event = 0x0000c8f9;
l2cap_request_connection_parameter_update = 0x0000c913;
l2cap_send_echo_request = 0x0000cded;
l2cap_unregister_service = 0x0000cead;
le_device_db_add = 0x0000cf05;
le_device_db_find = 0x0000cfdd;
le_device_db_from_key = 0x0000d009;
le_device_db_iter_cur = 0x0000d011;
le_device_db_iter_cur_key = 0x0000d015;
le_device_db_iter_init = 0x0000d019;
le_device_db_iter_next = 0x0000d021;
le_device_db_remove_key = 0x0000d047;
ll_ackable_packet_alloc = 0x0000d073;
ll_ackable_packet_get_status = 0x0000d1a5;
ll_ackable_packet_run = 0x0000d215;
ll_ackable_packet_set_tx_data = 0x0000d2b1;
ll_aes_encrypt = 0x0000d2cd;
ll_attach_cte_to_adv_set = 0x0000d349;
ll_channel_monitor_alloc = 0x0000d4e1;
ll_channel_monitor_check_each_pdu = 0x0000d563;
ll_channel_monitor_run = 0x0000d5c9;
ll_config = 0x0000d67d;
ll_free = 0x0000d6b3;
ll_get_heap_free_size = 0x0000d6bd;
ll_hint_on_ce_len = 0x0000d6d1;
ll_legacy_adv_set_interval = 0x0000d709;
ll_lock_frequency = 0x0000d719;
ll_malloc = 0x0000d77d;
ll_query_timing_info = 0x0000d8b5;
ll_raw_packet_alloc = 0x0000d901;
ll_raw_packet_free = 0x0000d9d5;
ll_raw_packet_get_bare_rx_data = 0x0000da0d;
ll_raw_packet_get_iq_samples = 0x0000dad3;
ll_raw_packet_get_rx_data = 0x0000db6d;
ll_raw_packet_recv = 0x0000dc0d;
ll_raw_packet_send = 0x0000dcc9;
ll_raw_packet_set_bare_data = 0x0000ddb1;
ll_raw_packet_set_bare_mode = 0x0000ddef;
ll_raw_packet_set_fake_cte_info = 0x0000def5;
ll_raw_packet_set_param = 0x0000df17;
ll_raw_packet_set_rx_cte = 0x0000df75;
ll_raw_packet_set_tx_cte = 0x0000e00b;
ll_raw_packet_set_tx_data = 0x0000e049;
ll_register_hci_acl_previewer = 0x0000e0ad;
ll_scan_set_fixed_channel = 0x0000e111;
ll_scanner_enable_iq_sampling = 0x0000e11d;
ll_set_adv_access_address = 0x0000e3d1;
ll_set_adv_coded_scheme = 0x0000e3dd;
ll_set_conn_acl_report_latency = 0x0000e40d;
ll_set_conn_coded_scheme = 0x0000e43d;
ll_set_conn_interval_unit = 0x0000e469;
ll_set_conn_latency = 0x0000e475;
ll_set_conn_tx_power = 0x0000e4a5;
ll_set_def_antenna = 0x0000e4ed;
ll_set_initiating_coded_scheme = 0x0000e509;
ll_set_max_conn_number = 0x0000e515;
ll_unlock_frequency = 0x0000e5a9;
nibble_for_char = 0x0001f869;
platform_calibrate_rt_clk = 0x0001f915;
platform_call_on_stack = 0x00004183;
platform_cancel_us_timer = 0x0001f919;
platform_config = 0x0001f92d;
platform_create_us_timer = 0x0001fa51;
platform_delete_timer = 0x0001fa65;
platform_enable_irq = 0x0001fa6d;
platform_get_current_task = 0x0001faa5;
platform_get_gen_os_driver = 0x0001fac9;
platform_get_heap_status = 0x0001fad1;
platform_get_link_layer_interf = 0x0001fae9;
platform_get_task_handle = 0x0001faf1;
platform_get_timer_counter = 0x0001fb11;
platform_get_us_time = 0x0001fb15;
platform_get_version = 0x0001fb19;
platform_hrng = 0x0001fb21;
platform_install_isr_stack = 0x0001fb29;
platform_install_task_stack = 0x0001fb35;
platform_patch_rf_init_data = 0x0001fb6d;
platform_printf = 0x0001fb79;
platform_raise_assertion = 0x0001fb8d;
platform_rand = 0x0001fba1;
platform_read_info = 0x0001fba5;
platform_read_persistent_reg = 0x0001fbd5;
platform_reset = 0x0001fbe5;
platform_rt_rc_auto_tune = 0x0001fc09;
platform_rt_rc_auto_tune2 = 0x0001fc11;
platform_rt_rc_tune = 0x0001fc99;
platform_set_abs_timer = 0x0001fcad;
platform_set_evt_callback = 0x0001fcb1;
platform_set_evt_callback_table = 0x0001fcc5;
platform_set_irq_callback = 0x0001fcd1;
platform_set_irq_callback_table = 0x0001fced;
platform_set_rf_clk_source = 0x0001fcf9;
platform_set_rf_init_data = 0x0001fd05;
platform_set_rf_power_mapping = 0x0001fd11;
platform_set_timer = 0x0001fd1d;
platform_shutdown = 0x0001fd21;
platform_switch_app = 0x0001fd25;
platform_trace_raw = 0x0001fd51;
platform_write_persistent_reg = 0x0001fd69;
printf_hexdump = 0x0001ff1d;
pvPortMalloc = 0x00020a11;
pvTaskIncrementMutexHeldCount = 0x00020af9;
pvTimerGetTimerID = 0x00020b11;
pxPortInitialiseStack = 0x00020b3d;
reverse_128 = 0x00020d1d;
reverse_24 = 0x00020d23;
reverse_256 = 0x00020d29;
reverse_48 = 0x00020d2f;
reverse_56 = 0x00020d35;
reverse_64 = 0x00020d3b;
reverse_bd_addr = 0x00020d41;
reverse_bytes = 0x00020d47;
sm_add_event_handler = 0x00021065;
sm_address_resolution_lookup = 0x000211bd;
sm_authenticated = 0x00021565;
sm_authorization_decline = 0x00021573;
sm_authorization_grant = 0x00021593;
sm_authorization_state = 0x000215b3;
sm_bonding_decline = 0x000215cd;
sm_config = 0x00021a29;
sm_config_conn = 0x00021a5d;
sm_encryption_key_size = 0x00021c17;
sm_just_works_confirm = 0x0002219d;
sm_le_device_key = 0x000224dd;
sm_passkey_input = 0x00022573;
sm_private_random_address_generation_get = 0x0002292d;
sm_private_random_address_generation_get_mode = 0x00022935;
sm_private_random_address_generation_set_mode = 0x00022941;
sm_private_random_address_generation_set_update_period = 0x00022969;
sm_register_external_ltk_callback = 0x00022aa5;
sm_register_oob_data_callback = 0x00022ab1;
sm_request_pairing = 0x00022abd;
sm_send_security_request = 0x0002359b;
sm_set_accepted_stk_generation_methods = 0x000235c1;
sm_set_authentication_requirements = 0x000235cd;
sm_set_encryption_key_size_range = 0x000235dd;
sscanf_bd_addr = 0x0002399d;
sysSetPublicDeviceAddr = 0x00023d51;
uuid128_to_str = 0x000244fd;
uuid_add_bluetooth_prefix = 0x00024555;
uuid_has_bluetooth_prefix = 0x00024575;
uxListRemove = 0x00024591;
uxQueueMessagesWaiting = 0x000245b9;
uxQueueMessagesWaitingFromISR = 0x000245e1;
uxQueueSpacesAvailable = 0x000245fd;
uxTaskGetStackHighWaterMark = 0x00024629;
uxTaskPriorityGet = 0x00024649;
uxTaskPriorityGetFromISR = 0x00024665;
vListInitialise = 0x0002471f;
vListInitialiseItem = 0x00024735;
vListInsert = 0x0002473b;
vListInsertEnd = 0x0002476b;
vPortEndScheduler = 0x00024785;
vPortEnterCritical = 0x000247b1;
vPortExitCritical = 0x000247f5;
vPortFree = 0x00024829;
vPortSuppressTicksAndSleep = 0x000248bd;
vPortValidateInterruptPriority = 0x000249e5;
vQueueDelete = 0x00024a41;
vQueueWaitForMessageRestricted = 0x00024a6d;
vTaskDelay = 0x00024ab5;
vTaskInternalSetTimeOutState = 0x00024b01;
vTaskMissedYield = 0x00024b11;
vTaskPlaceOnEventList = 0x00024b1d;
vTaskPlaceOnEventListRestricted = 0x00024b55;
vTaskPriorityDisinheritAfterTimeout = 0x00024b95;
vTaskPrioritySet = 0x00024c41;
vTaskResume = 0x00024d09;
vTaskStartScheduler = 0x00024d8d;
vTaskStepTick = 0x00024e1d;
vTaskSuspend = 0x00024e4d;
vTaskSuspendAll = 0x00024f09;
vTaskSwitchContext = 0x00024f19;
xPortStartScheduler = 0x00024fc9;
xQueueAddToSet = 0x00025091;
xQueueCreateCountingSemaphore = 0x000250b5;
xQueueCreateCountingSemaphoreStatic = 0x000250f1;
xQueueCreateMutex = 0x00025135;
xQueueCreateMutexStatic = 0x0002514b;
xQueueCreateSet = 0x00025165;
xQueueGenericCreate = 0x0002516d;
xQueueGenericCreateStatic = 0x000251b9;
xQueueGenericReset = 0x00025221;
xQueueGenericSend = 0x000252ad;
xQueueGenericSendFromISR = 0x00025419;
xQueueGiveFromISR = 0x000254d9;
xQueueGiveMutexRecursive = 0x0002557d;
xQueueIsQueueEmptyFromISR = 0x000255bd;
xQueueIsQueueFullFromISR = 0x000255e1;
xQueuePeek = 0x00025609;
xQueuePeekFromISR = 0x00025731;
xQueueReceive = 0x0002579d;
xQueueReceiveFromISR = 0x000258c9;
xQueueRemoveFromSet = 0x0002595d;
xQueueSelectFromSet = 0x0002597f;
xQueueSelectFromSetFromISR = 0x00025991;
xQueueSemaphoreTake = 0x000259a5;
xQueueTakeMutexRecursive = 0x00025b11;
xTaskCheckForTimeOut = 0x00025b55;
xTaskCreate = 0x00025bc5;
xTaskCreateStatic = 0x00025c21;
xTaskGetCurrentTaskHandle = 0x00025c91;
xTaskGetSchedulerState = 0x00025c9d;
xTaskGetTickCount = 0x00025cb9;
xTaskGetTickCountFromISR = 0x00025cc5;
xTaskIncrementTick = 0x00025cd5;
xTaskPriorityDisinherit = 0x00025da1;
xTaskPriorityInherit = 0x00025e35;
xTaskRemoveFromEventList = 0x00025ec9;
xTaskResumeAll = 0x00025f49;
xTaskResumeFromISR = 0x00026011;
xTimerCreate = 0x0002609d;
xTimerCreateStatic = 0x000260d1;
xTimerCreateTimerTask = 0x00026109;
xTimerGenericCommand = 0x00026175;
xTimerGetExpiryTime = 0x000261e5;
xTimerGetTimerDaemonTaskHandle = 0x00026205;
