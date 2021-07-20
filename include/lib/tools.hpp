#ifndef QUEUE_H
#define QUEUE_H
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace std;
typedef struct result_link_datatype
{
	struct result_node_datatype *head;
	struct result_node_datatype *end;
	int result_num;
}result_link_type;

/*
struct result_node_datatype
{
	cv::Mat result;
	struct result_node_datatype* next;
};
*/

//void result_push(result_link_type* result_link, result_node_datatype * result_node); //入队操作
//struct result_node_datatype* result_pop(result_link_type* result_link);//出队操作
//string get_time();
void safe_free(char *ptr);
#endif




