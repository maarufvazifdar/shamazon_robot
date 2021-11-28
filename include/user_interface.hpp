/**
 * BSD-3 Clause
 *
 * Copyright (c) 2021 Maaruf Vazifdar, Maitreya Kulkarni, Pratik Bhujnbal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the  
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE. 
 * 
 * @file user_interface.hpp
 * @author Maaruf Vazifdar
 * @author Maitreya Kulkarni
 * @author Pratik Bhujnbal
 * @brief Class to hold UserInterface attributes and members.
 * @version 1.0
 * @date 11/26/2021
 * @copyright  Copyright (c) 2021
 * 
 */

#pragma once
#include<string>


/**
 * @brief Class to hold UserInterface attributes and members.
 */
class UserInterface {
 public:
   int delivery_floor;
   std::string delivery_location;

  /** 
   * @brief  Constrctor of Class UserInterface to initialize delivery_floor 
   *         and delivery_location values.
   */
  UserInterface() {
    int delivery_floor = 1;
    std::string delivery_location = "Location 1";
  }

  /**
   * @brief Destructor of class UserInterface.
   */
  ~UserInterface() {}

  /**
   * @brief Set Delivery Location.
   * @param delivery_location std::string - Location to deliver the package.
   * @return void
   */
  void setDeliveryLocation(std::string delivery_location);

  /**
   * @brief Set Delivery Floor.
   * @param delivery_floor int - Floor to deliver the package.
   * @return void
   */
  void setDeliveryFloor(int delivery_floor);
};
