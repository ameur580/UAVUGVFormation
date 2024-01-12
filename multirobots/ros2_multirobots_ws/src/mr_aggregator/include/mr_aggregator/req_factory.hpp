#ifndef __REQ_FACTORY_HPP
#define __REQ_FACTORY_HPP

#include <string>

using namespace std;

template<class TRequest>
class ReqFactory {
	public:
		typedef TRequest Request; 
		virtual typename TRequest::SharedPtr get_request(int step, string &id)=0;
		virtual void display_req(string msg, typename TRequest::SharedPtr)=0;
};

#endif
