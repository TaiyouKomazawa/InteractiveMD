#ifndef VECTOR_3_MSG_HPP_
#define VECTOR_3_MSG_HPP_

#include <Message.hpp>

typedef struct Vector3MsgType
{
	float x;
	float y;
	float z;
} vector3_t;

typedef sb::Message<vector3_t> Vector3;

#endif
