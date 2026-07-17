#include "File_IO.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/**
 * @brief 从文本文件中读取一个字符串（不含空格，通常为数字）
 * @param path 文件路径
 * @param str  存储字符串的缓冲区（需足够大）
 * @return 0: 成功, -1: 失败
 * @note 此函数使用 fscanf 读取连续非空白字符
 */
int8_t file_read_string(const char *path, char *str)
{
    FILE *fp;
    int ret;

    fp = fopen(path, "r");
    if (fp == NULL) {
        printf("can not open file %s\r\n", path);
        return -1;
    }

    ret = fscanf(fp, "%s", str);
    if (ret != 1) {
        printf("file read error! ret=%d\r\n", ret);
        fclose(fp);
        return -1;
    }

    fclose(fp);
    return 0;
}

/**
 * @brief 将字符串写入文本文件（覆盖原内容）
 * @param path 文件路径
 * @param str  要写入的字符串
 * @return 0: 成功, -1: 失败
 */
int8_t file_write_string(const char *path, char *str)
{
    FILE *fp;

    fp = fopen(path, "w");
    if (fp == NULL) {
        printf("can not open file %s\r\n", path);
        return -1;
    }

    if (fputs(str, fp) == EOF) {
        perror("写入文件时出错");
        fclose(fp);
        return -1;
    }

    fclose(fp);
    return 0;
}