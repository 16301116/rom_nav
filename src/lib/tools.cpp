#include "lib/tools.hpp"
#include <iostream>
#include "time.h"

/*
void result_push(result_link_type* result_link, result_node_datatype * result_node) //入队操作
{
	if (result_link->head == NULL)
	{
		result_link->head = result_node;
		result_link->end = result_link->head;
		result_link->result_num++;
	}
	else
	{
		result_link->end->next = result_node;
		result_link->end = result_node;
		result_link->result_num++;
	}
}

struct result_node_datatype* result_pop(result_link_type* result_link) //出队操作
{
	struct result_node_datatype* tmp_node;
	if (result_link->head == NULL)
		return NULL;
	else if (result_link->head == result_link->end)
	{
		return NULL;
	}
	else
	{
		tmp_node = result_link->head;
		result_link->head = result_link->head->next;
		result_link->result_num--;
		return tmp_node;
	}
}


string get_time(){
		time_t t = time(0);
		struct tm *p;
		p=gmtime(&t);
		char s[100];
		strftime(s, sizeof(s), "%Y_%m_%d_%H_%M_%S_", p);
		//printf("%d: %s\n", (long)t, s); //1498124250: 2017-06-22 09:37:30
		ostringstream oss;
		oss<<s;
		return oss.str();
}

*/

void safe_free(char *ptr){
	if(ptr){
		free(ptr);
		ptr = NULL;
	}
}



