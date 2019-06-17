#include "scitos_mira/ModuleFactory.h"

#include "scitos_mira/ScitosCharger.h"
#include "scitos_mira/ScitosDisplay.h"
#include "scitos_mira/ScitosDrive.h"
#include "scitos_mira/ScitosEBC.h"
#include "scitos_mira/ScitosHead.h"
#include "scitos_mira/ScitosApplication.h"

ModuleFactory::ModuleFactory() {
	Register("Charger", &ScitosCharger::Create);
	Register("Display", &ScitosDisplay::Create);
	Register("Application", &ScitosApplication::Create);
	Register("Drive", &ScitosDrive::Create);
	Register("EBC", &ScitosEBC::Create);
	Register("Head", &ScitosHead::Create);
}

void ModuleFactory::Register(const std::string &name, ModuleCreator func) {
	modules_[name] = func;
}

ScitosModule *ModuleFactory::CreateModule(std::string name, ScitosG5 *robot)
{
	if (!CheckForModule(name))
	{
		ROS_ERROR("Trying to create unknown Scitos Module");
		return NULL;
	}
	std::map<std::string, ModuleCreator>::iterator it = modules_.find(name);
	ScitosModule *mod = it->second();
	mod->setRobot(robot);
	return mod;
}

bool ModuleFactory::CheckForModule(const std::string &name) {
  std::map<std::string, ModuleCreator>::iterator it = modules_.find(name);
  return it != modules_.end();
}
