#pragma once
#include<string>


class Elevator {
 public:
  Elevator() {
    int _elevator_floor = 1;
    bool _elevator_status = true;
  }

  ~Elevator() {}

  int getElevatorFloor();

  bool moveElevator(int delivery_floor);

 private:
  int _elevator_floor;
  bool _elevator_status;
};
