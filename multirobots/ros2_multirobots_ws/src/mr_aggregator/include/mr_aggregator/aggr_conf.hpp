#ifndef __AGGR_CONF_HPP
#define __AGGR_CONF_HPP

#include <string>

using namespace std;

namespace g_robot {
		class AggrConf{
			public:
				virtual string get_provide_data_service_name()=0;
				virtual string get_manage_service_name()=0;
		};
}
#endif
