# Leading object design doc

## Workflow

1. Find blocking objects on current lane path. Currently, we only consider stationary objects as blocking objects.
2. The leading objects are planner objects on current lane path, but not include objects labeled as blocking objects in step 1.

Following pictures show scenarios for leading objects. Refer to [link](https://docs.google.com/presentation/d/1vsSpK-WDR_ZM8wWjdd6jSznUqKPBwk_G9RqeyfJ6FEQ/edit#slide=id.gdc0c3a875f_0_0)

![leading_object_1](./img/leading_object_1.png)
![leading_object_2](./img/leading_object_2.png)
![leading_object_3](./img/leading_object_3.png)
![leading_object_4](./img/leading_object_4.png)
![leading_object_5](./img/leading_object_5.png)

3. If object is oncoming, not a leading object.
4. Consider object type: OT_VEHICLE, OT_UNKNOWN_MOVABLE, OT_MOTORCYCLIST.
