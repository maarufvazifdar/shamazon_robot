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
 * @file elevator.hpp
 * @author Maaruf Vazifdar
 * @author Maitreya Kulkarni
 * @author Pratik Bhujnbal
 * @brief Class to hold Elevator attributes and members.
 * @version 1.0
 * @date 11/26/2021
 * @copyright  Copyright (c) 2021
 * 
 */

#pragma once
#include<string>


/**
 * @brief Class to hold Elevator attributes and members.
 */
class Elevator {
 public:
  /** 
   * @brief  Constrctor of Class Elevator to initialize elevator floor 
   *         and  elevator status.
   */
  Elevator() {
    int _elevator_floor = 1;
    bool _elevator_status = true;
  }

  /**
   * @brief Destructor of class Elevator.
   */
  ~Elevator() {}

  /**
   * @brief Get current elevator floor.
   * @param void
   * @return _elevator_floor int - current elevator floor.
   */
  int getElevatorFloor();

  /**
   * @brief Move elevator to desired floor.
   * @param delivery_floor int - Floor to deliver the package.
   * @return bool - Status whether elevator reached desired floor.
   */
  bool moveElevator(int delivery_floor);

 private:
  /**
   * Class local variables.
   */
  int _elevator_floor;
  bool _elevator_status;
};
