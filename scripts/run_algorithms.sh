folder_path="../data_txt"
count=0
for file_path in $folder_path/*.txt; do
  # 计数器自增
  ((count++))
  # 循环遍历三个算法参数
  for algorithm in "madgwick" "mahony" "vqf"; do
    echo "Processing file $count: $file_path"
    # 调用 main 程序
    ../src/build/main "$file_path" "$algorithm" "6"
  done
done
