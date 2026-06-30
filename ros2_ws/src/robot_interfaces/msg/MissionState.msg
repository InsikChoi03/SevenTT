# FSM state + tray fill.
string state                    # SCAN | SELECT_TARGET | APPROACH | ALIGN | CLASSIFY | PICK |
                                # STORE_IN_TRAY | DRIVE_TO_STORAGE | ALIGN_OVER_BIN | DUMP_ALL | END
uint8 tray_shape_count          # set1 objects on tray
uint8 tray_fruit_count          # set2 objects on tray
uint64 current_target_id        # 0 if none
builtin_interfaces/Time stamp
