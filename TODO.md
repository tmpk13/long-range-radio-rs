- [ ] `SendingQueueIsFull` Error
    ```
    ...
    16:30:47.399: TX #181 failed: SendingQueueIsFull
    16:31:00.265: TX #182 failed: SendingQueueIsFull
    16:31:13.137: TX #183 failed: SendingQueueIsFull
    16:31:23.638: TX #184 failed: SendingQueueIsFull
    16:31:36.000: TX #185 failed: SendingQueueIsFul
    ...
    ```

- [ ] Separate radio logic from main

- [ ] Add basestation OTA node with uart and progress bar for sending out firmware updates
- [ ] Make sure the OTA updates will not be effected by interruptions.
- [x] Make it so i2c is polled periodically, not just hanging