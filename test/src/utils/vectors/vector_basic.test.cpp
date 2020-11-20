/*
 * Author: Pablo Morandé & Flori
 * Organisation: HYPED
 * Date: 13/11/2020
 * Description: Testing file for vector.cpp
 *
 *    Copyright 2018 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *    except in compliance with the License. You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software distributed under
 *    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *    either express or implied. See the License for the specific language governing permissions and
 *    limitations under the License.
 */
#include <math.h>
#include <cstdlib>
#include "gtest/gtest.h"
#include "utils/math/vector.hpp"


namespace hyped
{
namespace utils
{
namespace math
{
/**
 * @brief Helper method used to generate an array of three random integers.
 * each integer will be in the range -1000, 1000 (not inclusive)
 */
std::array<int, 3> createRandomArray()
{
  std::array<int, 3> output = std::array<int, 3>();
  for (int i = 0;i < 3;i++) {
    output[i] = rand()%1000;
  }
  return output;
}
/**
 * @brief Helper method used to generate an array of three random integers.
 * Also none of those Integers will be zero.
 * each integer will be in the range -1000, 1000 (not inclusive)
 */
std::array<int, 3> createRandomWithoutZeroesArray()
{
  std::array<int, 3> output = std::array<int, 3>();
  for (int i = 0;i < 3;i++) {
    output[i] = rand()%1000;
    while (output[i] == 0) {
      output[i] = rand()%1000;
    }
  }
  return output;
}
/**
 * @brief Test used to verify that the current implementation of the class vector handles the zero
 * argument constructor in an appropiate way.
 */
TEST(ConstructorTest, handlesZeroArgumentConstructor)
{
  const int dimension = 3;
  Vector<int, dimension> example = Vector<int, dimension>();
  ASSERT_EQ(0, example.norm());
  for (int i = 0;i < dimension;i++) {
    ASSERT_EQ(0, example[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector handles the
 * constructor of the form Vector<T,dimension>(T const), this should initialized all the values of
 * the vector to be equal to const.
 */
TEST(ConstructorTest, handlesConstantConstructor)
{
  const int element = 2;
  const int dimension = 3;
  Vector<int, dimension> example = Vector<int, dimension>(element);
  ASSERT_EQ(element*std::pow(dimension, 0.5), example.norm());
}
/**
 * @brief Test used to determine if the current implementation of the class vector handles the
 * constructor of the form Vector<T,dimension>(std::array<T,dimension> array),
 * this should assing each element of the vector with its correspondent element in the array
 */
TEST(ConstructorTest, handlesArrayContructor)
{
  const int dimension = 3;
  std::array<int, dimension> array =std::array<int, dimension>();
  array.fill(10);
  Vector<int, dimension> vector = Vector<int, dimension>(array);
  for (int i = 0;i < dimension;i++) {
    ASSERT_EQ(array[i], vector[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector handles the
 * constructor of the form Vector<T,dimension>(std::array<T,dimension> array). In this case the
 * elements of the array are not initialize so they should be zero, therefore the elements of the
 * vector should also be zero.
 */
TEST(ConstructorTest, handlesArrayContructorEmptyListTest)
{
  const int dimension = 3;
  std::array<int, dimension> array =std::array<int, dimension>();
  Vector<int, dimension> vector = Vector<int, dimension>(array);
  for (int i = 0;i < dimension;i++) {
    ASSERT_EQ(array[i], vector[i]);
    ASSERT_EQ(0, vector[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector handles the
 * constructor of the form Vector<T,dimension>(std::array<T> list),
 * this should assing each element of the vector with its correspondent element in the list
 */
TEST(ConstructorTest, handlesListConstructor)
{
  const int dimension = 3;
  std::initializer_list<int> list = std::initializer_list<int>({1, 2, 3});
  Vector<int, dimension> vector = Vector<int, dimension>(list);
  for (int i = 0;i < dimension;i++) {
    ASSERT_EQ(*(list.begin()+i), vector[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector handles the
 * constructor of the form Vector<T,dimension>(Vector<I,dimension> vector),
 * this should assing each element of the vector with its correspondent element in the other vector,
 * as they might not be of the same type fist the constructor should perform a cast.
 */
TEST(ConstructorTest, handlesDifferentTypeVector)
{
  const int dimension = 3;
  std::array<float, dimension> array_one =std::array<float, dimension>();
  array_one.fill(10.1);
  Vector<float, dimension> vector_one = Vector<float, dimension>(array_one);
  std::array<int, dimension> array_two =std::array<int, dimension>();
  array_two.fill(static_cast<int>(10.1));
  Vector<int, dimension> vector_two = Vector<int, dimension>(vector_one);
  for (int i = 0;i < dimension;i++) {
    ASSERT_EQ(array_two[i], vector_two[i]);
  }
}
/**
 * @brief Struct used to set up all the variables (vectors) used in the tests for basic operations
 * of vector with other vectors.
 * This tests will check if that basic operations are performed the way that they should. However,
 * they do not test properties.
 */
struct OperationsTest : public::testing::Test
{
  const int dimension = 3;
  std::array<int, 3> array_one;
  std::array<int, 3> array_two;
  Vector<int, 3> vector_one;
  Vector<int, 3> vector_two;
  void SetUp()
{
    array_one = createRandomArray();
    array_two = createRandomWithoutZeroesArray();
    vector_one = Vector<int, 3>(array_one);
    vector_two = Vector<int, 3>(array_two);
  }
};
/**
 * @brief Test used to determine if the current implementation of the class vector allows accessing
 * and modifying the elements of a vector in a normal way using [] operator.
 */
TEST_F(OperationsTest, allowsAccessAndModification)
{
  const int element = 10;
  const int new_element = 11;
  vector_one = Vector<int, 3>(element);
  float norm_one = vector_one.norm();
  vector_one[0] = new_element;
  float norm_new = vector_one.norm();
  ASSERT_NE(norm_one, norm_new);
  ASSERT_NE(vector_one[0], 10);
  ASSERT_EQ(vector_one[0], 11);
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows adding  a
 * vector to another vector in place in a normal way (element by element),
 * this tests the operator (+=).
 * The test will be performed using two random 3D vectors.
 */
TEST_F(OperationsTest, allowsAddition)
{
  vector_one+=vector_two;
  for (int i = 0;i <dimension;i++) {
    ASSERT_EQ(vector_one[i], vector_two[i]+array_one[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows adding in
 * a normal way (element by element), this tests the operator (+).
 * The test will be performed using two random 3D vectors.
 */
TEST_F(OperationsTest, allowsAutoAddition)
{
  vector_one = vector_one + vector_two;
  for (int i = 0;i <dimension;i++) {
    ASSERT_EQ(vector_one[i], vector_two[i] + array_one[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * substracting a vector to another vector in place in a normal way (element by element),
 * this tests the operator (-=).
 * The test will be performed using two random 3D vectors.
 */
TEST_F(OperationsTest, allowsSubstraction)
{
  vector_one-=vector_two;
  for (int i = 0;i <dimension;i++) {
    ASSERT_EQ(vector_one[i], -vector_two[i] + array_one[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * substracting a vector to another vector in place in a normal way (element by element),
 * this tests the operator (-).
 * The test will be performed using two random 3D vectors.
 */
TEST_F(OperationsTest, allowsAutoSubstraction)
{
  vector_one = vector_one - vector_two;
  for (int i = 0;i <dimension;i++) {
    ASSERT_EQ(vector_one[i], -vector_two[i] + array_one[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * multipliying a vector to another vector in place in a normal way (element by element),
 * this tests the operator (*=).
 * The test will be performed using two random 3D vectors.
 */
TEST_F(OperationsTest, allowsMultiplication)
{
  vector_one*=vector_two;
  for (int i = 0;i <dimension;i++) {
    ASSERT_EQ(vector_one[i], vector_two[i]*array_one[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * multipliying a vector to another vector in place in a normal way (element by element),
 * this tests the operator (*).
 * The test will be performed using two random 3D vectors.
 */
TEST_F(OperationsTest, allowsAutoMultiplication)
{
  vector_one = vector_one * vector_two;
  for (int i = 0;i <dimension;i++) {
    ASSERT_EQ(vector_one[i], vector_two[i]*array_one[i]);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * dividing a vector to another vector in a normal way (element by element),
 * this tests the operator (/).
 * The test will be performed using two random 3D vectors.
 */
TEST_F(OperationsTest, allowsAutoDivision)
{
  vector_one = vector_one / vector_two;
  for (int i = 0;i <dimension;i++) {
    ASSERT_EQ(vector_one[i], static_cast<int>(array_one[i] / vector_two[i]));
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * dividing a vector to another vector in place in a normal way (element by element),
 * this tests the operator (/=).
 * The test will be performed using two random 3D vectors.
 */
TEST_F(OperationsTest, allowsDivision)
{
  vector_one/=vector_two;
  for (int i = 0;i <dimension;i++) {
    ASSERT_EQ(vector_one[i], static_cast<int>(array_one[i]/vector_two[i]));
  }
}
/**
 * @brief Struct used to set up all the variables used in the tests for basic operationsof a vector
 * with constants.
 * This tests will check if that basic operations with constants are performed the way  they should.
 * However, they do not test properties.
 */
struct OperationsByConstant : public::testing::Test
{
  const int dimension = 3;
  Vector<int, 3> vector_one;
  Vector<int, 3> vector_two;
  Vector<int, 3> vector_three;
  Vector<int, 3> vector_result_one;
  Vector<int, 3> vector_result_two;
  int kValue = rand()%1000;
  void SetUp()
  {
    vector_one = Vector<int, 3>(createRandomArray());
    vector_two = Vector<int, 3>(createRandomArray());
    vector_three = Vector<int, 3>(createRandomArray());
    vector_result_one = Vector<int, 3>();
    vector_result_two = Vector<int, 3>();
  }
};
/**
 * @brief Test used to determine if the current implementation of the class vector allows adding in
 * a vector with a constant in normal way (adding the constant to each element),
 * this tests the operator (+).
 * The test will be performed using one random 3D vector and a random constant.
 */
TEST_F(OperationsByConstant, handlesAutoAdditionWithConstant)
{
  vector_result_one = vector_one + kValue;
  vector_result_two = kValue + vector_one;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], vector_one[i] + kValue);
    ASSERT_EQ(vector_result_two[i], vector_one[i] + kValue);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows adding
 * a vector with a constant in place (updating the initial vector) in a normal way
 * (adding the constant to each element), this tests the operator (+=).
 * The test will be performed using one random 3D vector and a random constant.
 */
TEST_F(OperationsByConstant, handlesAdditionWithConstant)
{
  vector_result_one += kValue;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], kValue);
  }
  vector_result_one += vector_one;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], vector_one[i] + kValue);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * substracting a constant to a vector in normal way (substracting the constant to each element),
 * this tests the operator (-).
 * The test will be performed using one random 3D vector and a random constant.
 */
TEST_F(OperationsByConstant, handlesAutoSubstractionWithConstant)
{
  vector_result_one = vector_one - kValue;
  vector_result_two = kValue - vector_one;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], vector_one[i] - kValue);
    ASSERT_EQ(vector_result_two[i], -vector_one[i] + kValue);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * substracting a constant to a vector in place (updating the initial vector)
 * in normal way (substracting the constant to each element),
 * this tests the operator (-=).
 * The test will be performed using one random 3D vector and a random constant.
 */
TEST_F(OperationsByConstant, handlesSubstractionWithConstant)
{
  vector_result_one -= kValue;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], -kValue);
  }
  vector_result_one += vector_one;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], vector_one[i] - kValue);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * multiplying a constant to a vector in normal way (multiplying the constant with each element),
 * this tests the operator (*).
 * The test will be performed using one random 3D vector and a random constant.
 */
TEST_F(OperationsByConstant, handlesAutoMultiplicationWithConstant)
{
  vector_result_one = vector_result_one * kValue;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], 0);
  }
  vector_result_one = vector_one * kValue;
  vector_result_two = kValue * vector_one;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], vector_one[i] * kValue);
    ASSERT_EQ(vector_result_two[i], vector_one[i] * kValue);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * multiplying a constant to a vector in place (updating the initial vector)
 * in normal way (multiplying the constant with each element),
 * this tests the operator (*=).
 * The test will be performed using one random 3D vector and a random constant.
 */
TEST_F(OperationsByConstant, handlesMultiplicationWithConstant)
{
  vector_result_one *= kValue;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], 0);
  }
  vector_result_one = Vector<int, 3>(1);
  vector_result_one *= kValue;
  vector_result_one *= vector_one;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], vector_one[i] * kValue);
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * multiplying a constant to a vector in normal way (dividing each element by the constant),
 * this tests the operator (=).
 * The test will be performed using one random 3D vector and a random constant.
 * It is important to be sure that the constant is not zero to avoid
 * division 0/0
 */
TEST_F(OperationsByConstant, handlesAutoDivisionWithConstant)
{
  while (kValue == 0) {
    kValue = rand() % 1000;
  }
  vector_result_one = vector_result_one / kValue;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], 0);
  }
  vector_result_one = vector_one / kValue;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], static_cast<int>(vector_one[i]/ kValue));
  }
}
/**
 * @brief Test used to determine if the current implementation of the class vector allows
 * multiplying a constant to a vector in place (updating the initial vector)
 * in normal way (dividing each element by the constant),
 * this tests the operator (/=).
 * The test will be performed using one random 3D vector and a random constant.
 * It is important to be sure that the constant is not zero to avoid
 * division 0/0
 */
TEST_F(OperationsByConstant, handlesDivisionWithConstant)
{
  while (kValue == 0) {
    kValue = rand() % 1000;
  }
  vector_result_one /= kValue;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], 0);
  }
  vector_result_one = vector_one;
  vector_result_one /= kValue;
  for (int i = 0; i < dimension; i++) {
    ASSERT_EQ(vector_result_one[i], static_cast<int>(vector_one[i]/ kValue));
  }
}
}
}
}