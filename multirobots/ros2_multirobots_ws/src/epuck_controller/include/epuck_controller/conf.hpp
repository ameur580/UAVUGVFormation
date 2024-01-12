#ifndef __CONF_HPP
#define __CONF_HPP

#include "mr_aggregator/aggr_conf.hpp"

class Conf: g_robot::AggrConf {
	public:
		string get_provide_data_service_name(){
			return "/averager/provide_data";
		}

		string get_manage_service_name(){
			return "/averager/manage";
		}
};

#endif
