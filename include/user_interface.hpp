#pragma once
#include<string>


class UserInterface {
 public:
   int delivery_floor;
   std::string delivery_location;

  UserInterface() {
    int delivery_floor = 1;
    std::string delivery_location = "Location 1";
  }

  ~UserInterface() {}

  void setDeliveryLocation(std::string delivery_location);

  void setDeliveryFloor(int delivery_floor);
};
