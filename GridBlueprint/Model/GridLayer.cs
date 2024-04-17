using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Mars.Components.Environments;
using Mars.Components.Layers;
using Mars.Core.Data;
using Mars.Interfaces.Data;
using Mars.Interfaces.Layers;

namespace GridBlueprint.Model;

public class GridLayer : RasterLayer, ISteppedActiveLayer
{
    #region Init

    /// <summary>
    ///     The initialization method of the GridLayer which spawns and stores the specified number of each agent type
    /// </summary>
    /// <param name="layerInitData"> Initialization data that is passed to an agent manager which spawns the specified
    /// number of each agent type</param>
    /// <param name="registerAgentHandle">A handle for registering agents</param>
    /// <param name="unregisterAgentHandle">A handle for unregistering agents</param>
    /// <returns>A boolean that states if initialization was successful</returns>
    public override bool InitLayer(LayerInitData layerInitData, RegisterAgent registerAgentHandle,
        UnregisterAgent unregisterAgentHandle)
    {
        var initLayer = base.InitLayer(layerInitData, registerAgentHandle, unregisterAgentHandle);
        RuleBasedAgentEnvironment = new SpatialHashEnvironment<RuleBasedAgent>(Width, Height);
        ReinforcementAgentEnvironment = new SpatialHashEnvironment<ReinforcementAgent>(Width, Height);
        // LoadQTable();
        var agentManager = layerInitData.Container.Resolve<IAgentManager>();
        RuleBasedAgents = agentManager.Spawn<RuleBasedAgent, GridLayer>().ToList();
        ReinforcementAgents = agentManager.Spawn<ReinforcementAgent, GridLayer>().ToList();
        HelperAgents = agentManager.Spawn<HelperAgent, GridLayer>().ToList();

        return initLayer;
    }

    #endregion

    #region Methods

    /// <summary>
    ///     Checks if the grid cell (x,y) is accessible
    /// </summary>
    /// <param name="x">x-coordinate of grid cell</param>
    /// <param name="y">y-coordinate of grid cell</param>
    /// <returns>Boolean representing if (x,y) is accessible</returns>
    public override bool IsRoutable(int x, int y) => this[x, y] == 0;

    #endregion

    #region Fields and Properties
    
    /// <summary>
    ///     The environment of the ReinforcementAgent agents
    /// </summary>
    public SpatialHashEnvironment<ReinforcementAgent> ReinforcementAgentEnvironment { get; set; }

    /// <summary>
    ///     The environment of the RuleBasedAgent agents
    /// </summary>
    public SpatialHashEnvironment<RuleBasedAgent> RuleBasedAgentEnvironment { get; set; }
    
    /// <summary>
    ///     A collection that holds the ReinforcementAgent instances
    /// </summary>
    public List<ReinforcementAgent> ReinforcementAgents { get; private set; }

    /// <summary>
    ///     A collection that holds the RuleBasedAgent instances
    /// </summary>
    public List<RuleBasedAgent> RuleBasedAgents { get; private set; }

    /// <summary>
    ///     A collection that holds the HelperAgent instance
    /// </summary>
    public List<HelperAgent> HelperAgents { get; private set; }

    private void LoadQTable()
    {
        var filePath = Path.Combine(Directory.GetParent(Environment.CurrentDirectory).Parent.FullName, 
            "Resources", "ReinforcementAgentQTable.json");
        if (File.Exists(filePath))
        {
            var json = File.ReadAllText(filePath);

            // var options = new JsonSerializerOptions { WriteIndented = true };
            //
            // var jaggedArray = JsonSerializer.Deserialize<double[][]>(json, options);
            // QTable = ToMultiDimensionalArray(jaggedArray);
        }
    }

    public double[,] QTable;

    #endregion

    public void Tick()
    {
        
    }

    public void PreTick()
    {
        
    }

    public void PostTick()
    {
        
    }
}