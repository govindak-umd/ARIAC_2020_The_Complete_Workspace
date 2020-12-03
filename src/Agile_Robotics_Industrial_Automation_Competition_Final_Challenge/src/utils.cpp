/**
 * @file utils.cpp
 * @author Pradeep Gopal, Govind Ajith Kumar, Rajesh NS, Cheng, Dakota Abernathy
 * @copyright MIT License
 * @brief Implementation of the Utils class
 * This class contains all the different utilities such as template structs that are needed by other classes
 */

/**
 *MIT License
 *Copyright (c) 2020 Pradeep Gopal, Govind Ajith Kumar, Rajesh NS, Cheng, Dakota Abernathy
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#include "utils.h"

/**
 * Hash-map to get different model heights for different parts.
 * They were modified because each part sinks into the surface a bit,
 * in varying amounts.
 */
std::unordered_map<std::string, double> model_height = { {
    "piston_rod_part_red", 0.0065 }, { "piston_rod_part_green", 0.0065 }, {
    "piston_rod_part_blue", 0.0065 }, { "pulley_part_red", 0.07 }, {
    "pulley_part_green", 0.07 }, { "pulley_part_blue", 0.07 }, {
    "gear_part_red", 0.012 }, { "gear_part_green", 0.012 }, { "gear_part_blue",
    0.012 }, { "gasket_part_red", 0.02 }, { "gasket_part_green", 0.02 }, {
    "gasket_part_blue", 0.02 }, { "disk_part_red", 0.023 }, { "disk_part_green",
    0.02 }, { "disk_part_blue", 0.023 } };
