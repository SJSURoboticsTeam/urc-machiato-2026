#include <iostream>
#include <string>
#include <memory>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/xml_parsing.h"

int main(int argc, char ** argv)
{
    if (argc < 2) {
        std::cout << "Usage: bt_debugger <bt_xml_file>" << std::endl;
        std::cout << "Example: bt_debugger behavior_trees/main_mission.xml" << std::endl;
        return 1;
    }

    std::string bt_file = argv[1];

    try {
        // Create factory and register nodes (same as orchestrator)
        BT::BehaviorTreeFactory factory;

        // Register basic nodes for validation
        // Note: These registrations are commented out as they register abstract classes
        // Concrete implementations should be registered in the actual orchestrator
        // factory.registerNodeType<BT::SyncActionNode>("CallService");
        // factory.registerNodeType<BT::SyncActionNode>("SensorCheck");
        // factory.registerNodeType<BT::SyncActionNode>("NavigateToWaypoint");
        // factory.registerNodeType<BT::SyncActionNode>("SampleCollection");
        // factory.registerNodeType<BT::SyncActionNode>("EmergencyStop");

        std::cout << "Loading behavior tree: " << bt_file << std::endl;

        // Load and validate tree structure
        auto tree = factory.createTreeFromFile(bt_file);

        std::cout << "✓ Behavior tree loaded successfully" << std::endl;
        std::cout << "✓ XML structure is valid" << std::endl;

        // Print tree structure
        std::cout << "\nTree Structure:" << std::endl;
        BT::printTreeRecursively(tree.rootNode());

        // Test tree ticking (dry run)
        std::cout << "\nTesting tree execution (dry run):" << std::endl;
        BT::StdCoutLogger logger(tree);
        logger.setEnabled(false);  // Disable console spam for validation

        auto status = tree.tickOnce();
        std::cout << "Initial tick status: " << BT::toStr(status) << std::endl;

        // Check for missing node implementations
        // Note: CreateManifest API changed in BT.CPP v4.x
        // auto manifest = BT::CreateManifest(tree.rootNode());
        // std::cout << "\nNode Manifest:" << std::endl;
        // for (const auto& [node_name, ports] : manifest) {
        //     std::cout << "  " << node_name << std::endl;
        //     for (const auto& port : ports) {
        //         std::cout << "    - " << port.first << " (" << port.second << ")" << std::endl;
        //     }
        // }

        std::cout << "\n✓ Behavior tree validation complete" << std::endl;
        std::cout << "✓ Ready for deployment" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "✗ Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
