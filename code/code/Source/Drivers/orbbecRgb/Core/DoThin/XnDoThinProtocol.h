#pragma once
#ifndef XN_DOTHIN_PROTOCOL 
#define XN_DOTHIN_PROTOCOL

#include "XnOS.h"
#include "XnMx6xOBCTeeFuns.h"


class XnDothinProtocol
{
public:
	XnDothinProtocol();

	~XnDothinProtocol();

public:

	XnStatus InitSpi(obc_ops_t* ops, XnInt dotinid);
	XnStatus InitI2C(obc_ops_t* ops, XnInt dotinid);
    XnStatus XnDothinProtocol::InitApOps(obc_ops_t *ops, XnInt dothin_id);

};

#endif //XN_DOTHIN_PROTOCOL



