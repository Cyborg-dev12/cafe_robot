



# State Machine Documentation

## States
- **Idle**: Waits for orders at home. Transitions to `MOVING_TO_KITCHEN` on `/orders` message.
- **MovingToKitchen**: Navigates to kitchen. Proceeds to `WAITING_AT_KITCHEN` or `RETURNING_TO_HOME` if canceled.
- **WaitingAtKitchen**: Waits for `/kitchen/confirmation`. Goes to `SELECT_NEXT_TABLE` or times out to `RETURNING_TO_HOME`.
- **SelectNextTable**: Picks next table from order list. Goes to `MOVING_TO_TABLE` or `RETURNING_TO_KITCHEN` if done.
- **MovingToTable**: Moves to selected table. Proceeds to `WAITING_AT_TABLE`.
- **WaitingAtTable**: Waits for table confirmation (e.g., `/table1/confirmation`). Returns to `SELECT_NEXT_TABLE`.
- **ReturningToKitchen**: Returns to kitchen after deliveries. Goes to `RETURNING_TO_HOME`.
- **ReturningToHome**: Returns to home position. Loops back to `IDLE`.

## Transitions
- See the assignment problem statement for detailed scenarios (e.g., timeouts, cancellations).
