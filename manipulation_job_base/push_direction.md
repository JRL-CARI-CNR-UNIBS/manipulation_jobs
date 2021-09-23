# Push in specific direction

## State descriptions
- Init:
- Prepared:
- Switch control:
- Simple Touch:
- Release:

## Transition descriptions

From | To | Condition |
------------ | ------------- | |
Init | Prepared | pre_execution server callback |
Prepared | Switch control | execution server callback |
Switch control | Simple Touch | --- |
Simple Touch | Return | wait for simple touch result |
Return | Init | wait for movement result |
