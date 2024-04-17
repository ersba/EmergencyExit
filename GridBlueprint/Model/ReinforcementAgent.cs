using System;
using System.Collections.Generic;
using System.Linq;
using Mars.Common;
using Mars.Interfaces.Agents;
using Mars.Interfaces.Annotations;
using Mars.Interfaces.Environments;
using Mars.Interfaces.Layers;
using Mars.Numerics;

namespace GridBlueprint.Model;

public class ReinforcementAgent : IAgent<GridLayer>, IPositionable
{
    #region Init

    /// <summary>
    ///     The initialization method of the ComplexAgent which is executed once at the beginning of a simulation.
    ///     It sets an initial Position and an initial State and generates a list of movement directions.
    /// </summary>
    /// <param name="layer">The GridLayer that manages the agents</param>
    public void Init(GridLayer layer)
    {
        _layer = layer;
        Position = new Position(StartX, StartY);
        _state = AgentState.MoveTowardsGoal;  // Initial state of the agent. Is overwritten eventually in Tick()
        _directions = CreateMovementDirectionsList();
        _layer.ReinforcementAgentEnvironment.Insert(this);
    }

    #endregion

    #region Tick

    /// <summary>
    ///     The tick method of the ComplexAgent which is executed during each time step of the simulation.
    ///     A ComplexAgent can move randomly along straight lines. It must stay within the bounds of the GridLayer
    ///     and cannot move onto grid cells that are not routable.
    /// </summary>
    public void Tick()
    {
        
        CalculateWall();
        Console.WriteLine(Position.X + " " + Position.Y);
    }

    #endregion

    #region Methods

    private int CalculateStateID()
    {
        int x = (int)Position.X;
        int y = (int)Position.Y;

        int max_x = _layer.Width;

        int state_id = y * max_x + x;
        return state_id;
    }

    private void CalculateWall()
    {
        var list = _layer.Explore(Position, 2, -1, cell => cell > 0);
        Console.WriteLine("break");
    }
    
    private int ChooseAction(double[] actionEstimates)
    {
        int length = actionEstimates.Length;
        double actionEstimate = actionEstimates[0];
        int num1 = 0;
        for (int index = 1; index < length; ++index)
        {
            if (actionEstimates[index] > actionEstimate)
            {
                actionEstimate = actionEstimates[index];
                num1 = index;
            }
        }
        if (_random.NextDouble() >= Epsilon)
            return num1;
        int num2 = _random.Next(length - 1);
        if (num2 >= num1)
            ++num2;
        return num2;
    }
    
    private void PerformAction(int action)
    {
        switch (action)
        {
            //Gehe zum Ziel
            case 0:
                
                break;
            //zu Hilfe eilen
            case 1:
                
                break;
            //kämpfen
            case 2:
                
                break;
            //fliehen
            case 3:

                break;
        }
    }

    
    private int CalculateReward(Position _currentState, Position _nextState, Position _goal)
    {
        var reward = 0;
    
        // Calculate distances from current and next states to the goal
        var currentDistance = CalculateDistance(_currentState, _goal);
        var nextDistance = CalculateDistance(_nextState, _goal);
    
        // If the agent moved towards the goal
        if (nextDistance < currentDistance)
        {
            reward += 100; // Increase reward for moving towards the goal
        }
        // If the agent moved away from the goal
        else if (nextDistance > currentDistance)
        {
            reward -= 100; // Decrease reward for moving away from the goal
        }
    
        // Other conditions
    
        // If the agent stayed in the same position
        if (_currentState.X == _nextState.X && _currentState.Y == _nextState.Y)
        {
            reward += 0; // No reward or penalty for staying in the same position
        }
    
        // If the agent's next position is occupied
        if (_layer.IsRoutable(_nextState.X, _nextState.Y) && _layer.ReinforcementAgentEnvironment.Explore(_nextState,
                radius: 1.0,
                predicate: agent => agent.Position.Equals(_nextState)).Any())
        {
            reward -= 50; // Penalty for trying to move to an occupied position
        }
    
        // If the agent cannot move due to an obstacle
        if (!_layer.IsRoutable(_nextState.X, _nextState.Y))
        {
            reward -= 100; // Penalty for hitting an obstacle
        }
        return reward;
    }
// Helper method to calculate the Euclidean distance between two positions
    private double CalculateDistance(Position position1, Position position2)
    {
        int deltaX = (int)(position2.X - position1.X);
        int deltaY = (int)(position2.Y - position1.Y);
        return Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private void UpdateQTable(int previousState, int chosenAction, int reward, int nextState)
    {
        double[] qvalue1 = _qTable[nextState];
        double num = qvalue1[0];
        for (int index = 1; index < 4; ++index)
        {
            if (qvalue1[index] > num)
                num = qvalue1[index];
        }
        double[] qvalue2 = _qTable[previousState];
        qvalue2[chosenAction] *= 1.0 - Alpha;
        qvalue2[chosenAction] += Alpha * (reward + Gamma * num);
    }

    /// <summary>
    ///     Generates a list of eight movement directions that the agent uses for random movement.
    /// </summary>
    /// <returns>The list of movement directions</returns>
    private static List<Position> CreateMovementDirectionsList()
    {
        return new List<Position>
        {
            MovementDirections.North,
            MovementDirections.Northeast,
            MovementDirections.East,
            MovementDirections.Southeast,
            MovementDirections.South,
            MovementDirections.Southwest,
            MovementDirections.West,
            MovementDirections.Northwest
        };
    }
    
    /// <summary>
    ///     Performs one random move, if possible, using the movement directions list.
    /// </summary>
    private void MoveRandomly()
    {
        var nextDirection = _directions[_random.Next(_directions.Count)];
        var newX = Position.X + nextDirection.X;
        var newY = Position.Y + nextDirection.Y;
        
        // Check if chosen move is within the bounds of the grid
        if (0 <= newX && newX < _layer.Width && 0 <= newY && newY < _layer.Height) 
        {
            // Check if chosen move goes to a cell that is routable and is empty
            if (_layer.IsRoutable(newX, newY) && !_layer.ReinforcementAgentEnvironment.Explore(Position, radius: 1.0, 
                    predicate: agent => agent.Position.Equals(new Position(newX, newY))).Any())
            {
                Position = new Position(newX, newY);
                _layer.ReinforcementAgentEnvironment.MoveTo(this, new Position(newX, newY));
                Console.WriteLine($"{GetType().Name} moved to a new cell: {Position}");
            }
            else
            {
                Console.WriteLine($"{GetType().Name} tried to move to a blocked cell: ({newX}, {newY})");
            }
        }
        else
        {
            Console.WriteLine($"{GetType().Name} tried to leave the world: ({newX}, {newY})");
        }
    }

    /// <summary>
    ///     Moves the agent towards a random routable adjacent cell via a calculated bearing.
    /// </summary>
    private void MoveWithBearing()
    {
        var goal = FindRoutableGoal();
        var bearing = PositionHelper.CalculateBearingCartesian(Position.X, Position.Y, goal.X, goal.Y);
        var curPos = Position;
        var newPos = _layer.ReinforcementAgentEnvironment.MoveTowards(this, bearing, 1);
        if (!_layer.IsRoutable(newPos))
        {
            Position = curPos;
            Console.WriteLine("Rollback");
        }
    }

    /// <summary>
    ///     Moves the agent one step along the shortest routable path towards a fixed goal.
    /// </summary>
    private void MoveTowardsGoal()
    {
        if (!_tripInProgress)
        {
            // Explore nearby grid cells based on their values
            _goal = new Position(49, _random.Next(1, 30));
            _path = _layer.FindPath(Position, _goal).GetEnumerator();
            _tripInProgress = true;
            _path.MoveNext();
        }

        
        if (_path.MoveNext())
        {
            if (!_layer.ReinforcementAgentEnvironment.Explore(Position, AgentExploreRadius,
                    predicate: agent => agent.Position.Equals(_path.Current)).Any())
            {
                _layer.ReinforcementAgentEnvironment.MoveTo(this, _path.Current, 1);
            }
            else
            {
                _tripInProgress = false;
            }
            if (Position.Equals(_goal))
            {
                Console.WriteLine($"ComplexAgent {ID} reached goal {_goal}");
                
                _goalReached = true;
            }
        }
    }

    /// <summary>
    ///     Finds a routable grid cell that serves as a goal for subsequent pathfinding.
    /// </summary>
    /// <param name="maxDistanceToGoal">The maximum distance in grid cells between the agent's position and its goal</param>
    /// <returns>The found grid cell</returns>
    private Position FindRoutableGoal(double maxDistanceToGoal = 1.0)
    {
        var nearbyRoutableCells = _layer.Explore(Position, radius: maxDistanceToGoal, predicate: cellValue => cellValue == 0.0).ToList();
        var goal = nearbyRoutableCells[_random.Next(nearbyRoutableCells.Count)].Node.NodePosition;

        // in case only one cell is routable, use directly no need to random!
        // other vise, try to find a cell we are not coming from
        if (nearbyRoutableCells.Count > 1)
        {
            while (Position.Equals(goal))
            {
                goal = nearbyRoutableCells[_random.Next(nearbyRoutableCells.Count)].Node.NodePosition;
            }
        }
        
        Console.WriteLine($"New goal: {goal}");
        return goal;
    }

    /// <summary>
    ///     Explores the environment for other agents and increments their counter if they are nearby.
    /// </summary>
    private void ExploreAgents()
    {
        // Explore nearby other ComplexAgent instances
        var agents = _layer.ReinforcementAgentEnvironment.Explore(Position, radius: AgentExploreRadius);

        foreach (var agent in agents)
        {
            if (Distance.Chebyshev(new []{Position.X, Position.Y}, new []{agent.Position.X, agent.Position.Y}) <= 1.0)
            {
                Console.WriteLine($"ComplexAgent {ID} found another ComplexAgent at {agent.Position}");
            }
        }
    }

    /// <summary>
    ///     Selects a new state from the AgentState enumeration to guide for subsequent behavior.
    ///     Will return the current state if a route is still in progress.
    /// </summary>
    /// <returns>The selected state</returns>
    private AgentState RandomlySelectNewState()
    {
        if (_state == AgentState.MoveTowardsGoal && _tripInProgress)
        {
            Console.WriteLine("Trip still in progress, so no state change.");
            return AgentState.MoveTowardsGoal;
        }

        var agentStates = Enum.GetValues(typeof(AgentState));
        var newState = (AgentState) agentStates.GetValue(_random.Next(agentStates.Length))!;
        Console.WriteLine($"New state: {newState}");
        return newState;
    }

    /// <summary>
    ///     Removes this agent from the simulation and, by extension, from the visualization.
    /// </summary>
    private void RemoveFromSimulation()
    {
        Console.WriteLine($"ComplexAgent {ID} is removing itself from the simulation.");
        _layer.ReinforcementAgentEnvironment.Remove(this);
        UnregisterAgentHandle.Invoke(_layer, this);
    }

    #endregion

    #region Fields and Properties

    public Guid ID { get; set; }
    
    public Position Position { get; set; }
    
    [PropertyDescription(Name = "Alpha")]
    // Learning rate
    public int Alpha { get; set; }
    
    [PropertyDescription(Name = "Gamma")]
    // Discount factor
    public int Gamma { get; set; }
    
    [PropertyDescription(Name = "Epsilon")]
    // Exploration rate
    public int Epsilon { get; set; }

    [PropertyDescription(Name = "StartX")]
    public int StartX { get; set; }
    
    [PropertyDescription(Name = "StartY")]
    public int StartY { get; set; }
    
    [PropertyDescription(Name = "MaxTripDistance")]
    public double MaxTripDistance { get; set; }
    
    [PropertyDescription(Name = "AgentExploreRadius")]
    public double AgentExploreRadius { get; set; }
    
    public UnregisterAgent UnregisterAgentHandle { get; set; }

    private Double[][] _qTable;
    private GridLayer _layer;
    private List<Position> _directions;
    private readonly Random _random = new();
    private Position _goal;
    private bool _tripInProgress;
    private AgentState _state;
    private List<Position>.Enumerator _path;
    private bool _moved = true;
    private bool _goalReached = false;
    #endregion
}