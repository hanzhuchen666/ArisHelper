﻿#include <aris.hpp>

int sendRequest(int argc, char *argv[])
{
	// 需要去除命令名的路径和扩展名 //
	std::string cmdName(argv[0]);

#ifdef WIN32
	if (cmdName.rfind('\\'))
	{
		cmdName = cmdName.substr(cmdName.rfind('\\') + 1, cmdName.npos);
	}
#endif
#ifdef UNIX
	if (cmdName.rfind('/'))
	{
		cmdName = cmdName.substr(cmdName.rfind('/') + 1, cmdName.npos);
	}
#endif

	// 去掉命令后缀,特别是在Windows系统下
	if (cmdName.rfind('.'))cmdName = cmdName.substr(0, cmdName.rfind('.'));

	// 添加命令的所有参数 //
	for (int i = 1; i < argc; ++i)cmdName = cmdName + " " + argv[i];

	// 构造msg,这里需要先copy命令名称,然后依次copy各个参数 //
	aris::core::Msg msg(0);
	msg.copy(cmdName.c_str());

	// 连接并发送msg //
	aris::core::Socket client("client");
	client.setConnectType(aris::core::Socket::TCP);
    client.setRemoteIP("127.0.0.1");
    client.setPort("5868");

	std::atomic_bool is_finished = false;
	client.setOnLoseConnection([&](aris::core::Socket *s)->int 
	{
		std::cout << "connection brocken" << std::endl;
		is_finished = true;
		return 0;
	});
	client.setOnReceivedMsg([&](aris::core::Socket *s, aris::core::Msg &msg)->int
	{
		std::cout <<"ret value:"<< msg.toString() << std::endl;
		is_finished = true;
		return 0;
	});
	
	// connect //
	for(bool c=false;!c;)
	{
		try
		{
			client.connect();
			c = true;
		}
		catch (std::exception &e)
		{
			std::cout << "failed to connect server, will retry in 1 second" << std::endl;
			std::cout << e.what() << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(1));
			continue;
		}
	}

	// send //
	try
	{
		client.sendMsg(msg);
		std::cout << "send cmd:" << msg.toString() << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
		return 0;
	}

	// wait //
	while (!is_finished) std::this_thread::sleep_for(std::chrono::milliseconds(1));

	return 0;
}

int main(int argc, char *argv[])
{
	if (argc <= 1)throw std::runtime_error("please input the cmd name");

	sendRequest(argc - 1, argv + 1);

	return 0;
}
