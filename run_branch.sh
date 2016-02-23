#!/bin/bash
echo "Running!"
# ./LB-ILS 3 1 3.0 0.75 2.0 1 < instances/AP40.txt > test_results/AP40/AP40_3_1-2.txt
# echo "3_1...ok!"
# ./LB-ILS 3 2 3.0 0.75 2.0 1 < instances/AP40.txt > test_results/AP40/AP40_3_2-2.txt
# echo "3_2...ok!"
# ./LB-ILS 3 3 3.0 0.75 2.0 1 < instances/AP40.txt > test_results/AP40/AP40_3_3-2.txt
# echo "3_3...ok!"
# ./LB-ILS 4 1 3.0 0.75 2.0 1 < instances/AP40.txt > test_results/AP40/AP40_4_1-2.txt
# echo "4_1...ok!"
# ./LB-ILS 4 2 3.0 0.75 2.0 1 < instances/AP40.txt > test_results/AP40/AP40_4_2-2.txt
# echo "4_2...ok!"
# ./LB-ILS 4 3 3.0 0.75 2.0 1 < instances/AP40.txt > test_results/AP40/AP40_4_3-2.txt
# echo "4_3...ok!"
# ./LB-ILS 4 4 3.0 0.75 2.0 1 < instances/AP40.txt > test_results/AP40/AP40_4_4-2.txt
# echo "4_4...ok!"
./LB-ILS 5 1 1.0 0.2 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_1_02-2.txt
echo "5_1_02...ok!"
./LB-ILS 5 1 1.0 0.4 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_1_04-2.txt
echo "5_1_04...ok!"
./LB-ILS 5 1 1.0 0.6 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_1_06-2.txt
echo "5_1_06...ok!"
./LB-ILS 5 1 1.0 1.0 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_1_10-2.txt
echo "5_1_10...ok!"
./LB-ILS 5 2 1.0 0.4 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_2_04-2.txt
echo "5_2_04...ok!"
./LB-ILS 5 2 1.0 0.6 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_2_06-2.txt
echo "5_2_06...ok!"
./LB-ILS 5 3 1.0 0.4 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_3_04-2.txt
echo "5_3_04...ok!"
./LB-ILS 5 3 1.0 0.6 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_3_06-2.txt
echo "5_3_06...ok!"
./LB-ILS 5 3 1.0 1.0 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_3_10-2.txt
echo "5_3_10...ok!"
./LB-ILS 5 5 1.0 0.2 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_5_02-2.txt
echo "5_5_02...ok!"
./LB-ILS 5 5 1.0 0.4 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_5_04-2.txt
echo "5_5_04...ok!"
./LB-ILS 5 5 1.0 0.6 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_5_06-2.txt
echo "5_5_06...ok!"
./LB-ILS 5 5 1.0 0.8 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_5_08-2.txt
echo "5_5_08...ok!"
./LB-ILS 5 5 1.0 1.0 1.0 1 < instances/CAB25.txt > test_results/CAB25_5_5_10-2.txt
echo "5_5_10...ok!"