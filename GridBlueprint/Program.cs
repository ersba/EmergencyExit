using System;
using System.IO;
using GridBlueprint.Model;
using Mars.Components.Starter;
using Mars.Core.Simulation.Entities;
using Mars.Interfaces.Model;

namespace GridBlueprint;

internal static class Program
{
    private static void Main()
    {
        // Create a new model description and add model components to it
        var description = new ModelDescription();
        description.AddLayer<GridLayer>();
        description.AddAgent<RuleBasedAgent, GridLayer>();
        description.AddAgent<ReinforcementAgent, GridLayer>();
        description.AddAgent<HelperAgent, GridLayer>();

        // Load the simulation configuration from a JSON configuration file
        var file = File.ReadAllText("config.json");
        var config = SimulationConfig.Deserialize(file);

        // // Couple model description and simulation configuration
        // var starter = SimulationStarter.Start(description, config);
        //
        // // Run the simulation
        // SimulationWorkflowState handle = starter.Run();
        //
        // // Close the program
        // Console.WriteLine("Successfully executed iterations: " + handle.Iterations);
        // starter.Dispose();
        for (var i = 1; i <= 1; i++)
        {
            var starter = SimulationStarter.Start(description, config);
            var handle = starter.Run();
            Console.WriteLine($"\nSuccessfully executed {handle.Iterations} iterations: Game: {i}\n");
            starter.Dispose();
            //GC.Collect();
        }
    }
}