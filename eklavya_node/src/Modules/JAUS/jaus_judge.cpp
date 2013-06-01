#include "jaus.h"

void *jaus_judge(void *arg) {
    // Create a component.  By default a component
    // has all the services of the Core Service set:
    // - Transport (JUDP)
    // - Control
    // - Discovery
    // - Events
    // - Liveness
    // - Time
    // - Management
    JAUS::Component component;

    // The Transport service is used to send
    // and receive messages to other JAUS components.  All
    // other services use the Transport service.  The
    // default transport type for JAUS++ is UDP communication
    // using the JUDP class.

    // The Discovery service is used to find
    // other JAUS components and services on the 
    // network using the Transport service.  In JAUS++
    // Discovery will automatically find these components,
    // make connections to them, and keep track of what
    // services they have.

    //  The first thing we must do for a component is
    //  configure its identification.  This is done by
    //  using the Discovery Service.  Get a pointer
    //  to the service:
    JAUS::Discovery* discoveryService = NULL;
    discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
    
    // Unable to set a custom IP Address
    ((JAUS::JUDP *)component.TransportService())->SetInterfaceIP("127.0.0.3");
    
    //  Alternative method:
    //  discoveryService = component.DiscoveryService();

    // Set the type of subsystem the component is for.  Subsystem
    // types available are currently Vehicle, or OCU.  The string
    // name "Robot" represents the type or category of platform.
    // You must set the subsystem identification before you will be
    // able to initialize your component.
    discoveryService->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,
            "Judge");
    // You can also set identification information for the component
    // and node that it is on.
    discoveryService->SetNodeIdentification("Desktop");
    discoveryService->SetComponentIdentification("Server");

    // Now that we have setup our identification information we
    // can initialize our component.  First, create the
    // component ID.
    JAUS::Address componentID(42, 1, 1);
    // Initialize!
    std::cout << "Initializing component...";
    if (component.Initialize(componentID) == false) {
        std::cout << "Failed to initialize component [" << componentID.ToString() << "]\n";
        return 0;
    }
    std::cout << "Success!\n";

    // Now go into your main computer loop until the
    // component has been told to shutdown.
    JAUS::Time::Stamp displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
    while (true) {
        // Let's check the "state" of our component. This
        // is done using the Management service.  
        // A component can be in the following states:
        // - Initialized
        // - Ready
        // - Standby
        // - Shutdown
        // - Failure
        // - Emergency
        JAUS::Management* managementService = NULL;
        managementService = (JAUS::Management*)component.GetService(JAUS::Management::Name);
        // Alternative method:
        //managementService = component.ManagementService();
        if (managementService->GetStatus() == JAUS::Management::Status::Shutdown) {
            // Exit program.
            break;
        }
        if (JAUS::Time::GetUtcTimeMs() - displayStatusTimeMs > 500) {
            std::cout << "======================================================\n";
            // Print status of the service.
            managementService->PrintStatus();
            std::cout << std::endl;

            displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
        }

        if (CxUtils::GetChar() == 27) {
            break;
        }

        CxUtils::SleepMs(1);
    }

    // Shutdown your component completely.  Any
    // services added or belonging to the component
    // will be deleted.
    component.Shutdown();

    return 0;
}

