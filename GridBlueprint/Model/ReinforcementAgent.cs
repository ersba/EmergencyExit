using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using Mars.Common;
using Mars.Common.IO;
using Mars.Interfaces.Agents;
using Mars.Interfaces.Annotations;
using Mars.Interfaces.Environments;
using Mars.Interfaces.Layers;
using Mars.Numerics;
using ServiceStack;

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
        _goal = new Position(ExitPositionX, ExitPositionY);
        if (File.Exists(_qTablePath))
        {
            var loadedQBytes = LoadFileToByteArrayWithFileStream(_qTablePath);
            var qObject = ObjectSerialize.DeSerialize(loadedQBytes);
            _qTable = qObject.ConvertTo<Double[][]>();
        }
        else
        {
            int numActions = 9;
            int width = _layer.Width;
            int height = _layer.Height;
            int numStates = width * height;
            _qTable = new double[numStates][];
            for (int index = 0; index < numStates; ++index)
                _qTable[index] = new double[numActions];
            for (int index1 = 0; index1 < numStates; ++index1)
            {
                for (int index2 = 0; index2 < numActions; ++index2)
                    _qTable[index1][index2] = 0;
            }
        }
        Position = new Position(StartX, StartY);
        _directions = CreateMovementDirectionsList();
        _currentState = CalculateCurrentState();
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
        if (_layer.GetCurrentTick() != 1)
        {
            _reward = CalculateReward();
            UpdateQTable(_previousState, _chosenAction, _reward, _currentState);
        }
        _chosenAction = ChooseAction(_qTable[_currentState]);
        _previousPosition = Position;
        _previousState = _currentState;
        _previousDirection = _direction;
        PerformAction(_chosenAction);
        _currentState = CalculateCurrentState();
        _goalReached = Position.Equals(_goal);
        if (Position.Equals(_goal))
        {
            _goalReached = !_goalReached;
        }

        if (_layer.GetCurrentTick() == 300)
        {
            SaveTable();
        }
        // Console.WriteLine(Position.X + " " + Position.Y);
    }

    #endregion

    #region Methods
    
    private void SaveTable()
    {
        if (StartX != 1 || StartY != 1)
        {
            return;
        }
        if (File.Exists(_qTablePath)) 
        {
            File.Delete(_qTablePath);
        } 
        var qBytes = _qTable.Serialize(); 
        SaveByteArrayToFileWithFileStream(qBytes, _qTablePath);
    }
    
    private DirectionType GetGoalDirection()
    {
        var deltaX = _goal.X - Position.X;
        var deltaY = _goal.Y - Position.Y;
        if (deltaX > 0)
        {
            if (deltaY > 0)
            {
                return DirectionType.UpRight;
            }
            if (deltaY < 0)
            {
                return DirectionType.DownRight;
            }
            return DirectionType.Right;
        }
        if (deltaX < 0)
        {
            if (deltaY > 0)
            {
                return DirectionType.UpLeft;
            }
            if (deltaY < 0)
            {
                return DirectionType.DownLeft;
            }
            return DirectionType.Left;
        }
        if (deltaY > 0)
        {
            return DirectionType.Up;
        }
        if (deltaY < 0)
        {
            return DirectionType.Down;
        }
        return DirectionType.NotSet;
    } 
    
    
    
    private byte[] LoadFileToByteArrayWithFileStream(string filePath)
    {
        return File.ReadAllBytes(filePath);
    }
    
    private void SaveByteArrayToFileWithFileStream(byte[] data, string filePath)
    {
        using var stream = File.Create(filePath);
        stream.Write(data, 0, data.Length);
    }

    private int CalculateCurrentState()
    {
        int x = (int)Position.X;
        int y = (int)Position.Y;

        int max_x = _layer.Width;

        int state_id = y * max_x + x;
        return state_id;
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
        // if (num2 >= num1)
        //     ++num2;
        return num2;
    }
    
    private void PerformAction(int action)
    {
        switch (action)
        {
            case 0:
                _direction = MovementDirections.North;
                break;
            case 1:
                _direction = MovementDirections.Northeast;
                break;
            case 2:
                _direction = MovementDirections.East;
                break;
            case 3:
                _direction = MovementDirections.Southeast;
                break;

            case 4:
                _direction = MovementDirections.South;
                break;

            case 5:
                _direction = MovementDirections.Southwest;
                break;

            case 6:
                _direction = MovementDirections.West;
                break;

            case 7:
                _direction = MovementDirections.Northeast;
                break;

            case 8:
                _direction = new Position(0, 0);
                break;
            
        }
        var newX = Position.X + _direction.X;
        var newY = Position.Y + _direction.Y;
        
        // Check if chosen move is within the bounds of the grid
        if (0 <= newX && newX < _layer.Width && 0 <= newY && newY < _layer.Height) 
        {
            // Check if chosen move goes to a cell that is routable and is empty
            if (_layer.IsRoutable(newX, newY) && !_layer.ReinforcementAgentEnvironment.Explore(Position, radius: 2.0, 
                    predicate: agent => agent.Position.Equals(new Position(newX, newY))).Any())
            {
                Position = new Position(newX, newY);
                _layer.ReinforcementAgentEnvironment.MoveTo(this, new Position(newX, newY));
                // Console.WriteLine($"{GetType().Name} moved to a new cell: {Position}");
            }
            else
            {
                // Console.WriteLine($"{GetType().Name} tried to move to a blocked cell: ({newX}, {newY})");
            }
        }
        else
        {
            // Console.WriteLine($"{GetType().Name} tried to leave the world: ({newX}, {newY})");
        }
    }

    
    private int CalculateReward()
    {
        var reward = 0;
        
        // Calculate distances from current and next states to the goal
        var previousDistance = CalculateDistance(_previousPosition, _goal);
        var currentDistance = CalculateDistance(Position, _goal);
    
        // If the agent moved towards the goal
        if (currentDistance < previousDistance && !_goalReached)
        {
            reward += 100; // Increase reward for moving towards the goal
        }
        // If the agent moved away from the goal
        else if (currentDistance > previousDistance  && !_goalReached)
        {
            reward -= 100; // Decrease reward for moving away from the goal
        }
    
        // Other conditions
    
        // If the agent stayed in the same position TO DO: Check if occupied
        if (Position.X == _previousPosition.X && Position.Y == _previousPosition.Y)
        {
            reward -= 0; // No reward or penalty for staying in the same position
                         // Changed to zero for now
        }
    
        // If the agent cannot move due to an obstacle
        if (!_layer.IsRoutable(_previousPosition.X, _previousPosition.Y))
        {
            reward -= 100; // Penalty for hitting an obstacle
        }
        
        // If the agent moved towards the goal even though he passed the goal
        if (currentDistance < previousDistance && _goalReached)
        {
            reward -= 100; // Increase reward for moving towards the goal
        }
        
        if (_goalReached && GetOppositeDirections(_previousDirection).Contains(_direction))
        {
            reward -= 1000;
        }
        
        if (currentDistance > previousDistance && _goalReached)
        {
            reward += 100; // Increase reward for moving towards the goal
        }
            
        return reward;
    }

    private List<Position> GetOppositeDirections(Position direction)
    {
        // Give me a list of the opposite directions for direction for instance direction MovementDirections.West, should return {MovementDirections.East, MovementDirections.Southeast, MovementDirections.Northeast}
        var oppositeDirections = new List<Position>();
        if (direction.Equals(MovementDirections.North))
        {
            oppositeDirections.Add(MovementDirections.South);
            oppositeDirections.Add(MovementDirections.Southwest);
            oppositeDirections.Add(MovementDirections.Southeast);
        }
        else if (direction.Equals(MovementDirections.Northeast))
        {
            oppositeDirections.Add(MovementDirections.Southwest);
            oppositeDirections.Add(MovementDirections.South);
            oppositeDirections.Add(MovementDirections.West);
        }
        else if (direction.Equals(MovementDirections.East))
        {
            oppositeDirections.Add(MovementDirections.West);
            oppositeDirections.Add(MovementDirections.Southwest);
            oppositeDirections.Add(MovementDirections.Northwest);
        }
        else if (direction.Equals(MovementDirections.Southeast))
        {
            oppositeDirections.Add(MovementDirections.Northwest);
            oppositeDirections.Add(MovementDirections.West);
            oppositeDirections.Add(MovementDirections.North);
        }
        else if (direction.Equals(MovementDirections.South))
        {
            oppositeDirections.Add(MovementDirections.North);
            oppositeDirections.Add(MovementDirections.Northwest);
            oppositeDirections.Add(MovementDirections.Northeast);
        }
        else if (direction.Equals(MovementDirections.Southwest))
        {
            oppositeDirections.Add(MovementDirections.Northeast);
            oppositeDirections.Add(MovementDirections.North);
            oppositeDirections.Add(MovementDirections.East);
        }
        else if (direction.Equals(MovementDirections.West))
        {
            oppositeDirections.Add(MovementDirections.East);
            oppositeDirections.Add(MovementDirections.Northeast);
            oppositeDirections.Add(MovementDirections.Southeast);
        }
        else if (direction.Equals(MovementDirections.Northwest))
        {
            oppositeDirections.Add(MovementDirections.Southeast);
            oppositeDirections.Add(MovementDirections.East);
            oppositeDirections.Add(MovementDirections.South);
        }
        return oppositeDirections;
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
        // double[] qvalue_old = qvalue1.Copy();
        double num = qvalue1[0];
        for (int index = 1; index < 4; ++index)
        {
            if (qvalue1[index] > num)
                num = qvalue1[index];
        }
        double[] qvalue2 = _qTable[previousState];
        qvalue2[chosenAction] *= 1.0 - Alpha;
        qvalue2[chosenAction] += Alpha * (reward + Gamma * num);
        // Console.WriteLine(qvalue1.Equals(qvalue_old));
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

    private void MoveNorth()
    {
        var newX = Position.X + MovementDirections.North.X;
        var newY = Position.Y + MovementDirections.North.Y;
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
                // Console.WriteLine($"{GetType().Name} moved to a new cell: {Position}");
            }
            else
            {
                // Console.WriteLine($"{GetType().Name} tried to move to a blocked cell: ({newX}, {newY})");
            }
        }
        else
        {
            // Console.WriteLine($"{GetType().Name} tried to leave the world: ({newX}, {newY})");
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
            // Console.WriteLine("Rollback");
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
        
        // Console.WriteLine($"New goal: {goal}");
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
                // Console.WriteLine($"ComplexAgent {ID} found another ComplexAgent at {agent.Position}");
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
            // Console.WriteLine("Trip still in progress, so no state change.");
            return AgentState.MoveTowardsGoal;
        }

        var agentStates = Enum.GetValues(typeof(AgentState));
        var newState = (AgentState) agentStates.GetValue(_random.Next(agentStates.Length))!;
        // Console.WriteLine($"New state: {newState}");
        return newState;
    }

    /// <summary>
    ///     Removes this agent from the simulation and, by extension, from the visualization.
    /// </summary>
    private void RemoveFromSimulation()
    {
        // Console.WriteLine($"ComplexAgent {ID} is removing itself from the simulation.");
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
    
    [PropertyDescription(Name = "ExitPositionX")]
    // Exit position X
    public int ExitPositionX { get; set; }
    
    [PropertyDescription(Name = "ExitPositionY")]
    // Exit position Y
    public int ExitPositionY { get; set; }

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
    private Position _previousPosition;
    private bool _tripInProgress;
    private AgentState _state;
    private int _currentState;
    private List<Position>.Enumerator _path;
    private bool _moved = true;
    private bool _goalReached = false;
    private int _chosenAction;
    private Position _direction;
    private int _previousState;
    private int _reward;
    private Position _previousDirection;
    private static Mutex mut = new Mutex();
    private string _qTablePath = "../../../Resources/ReinforcementAgent_QTable";

    #endregion
}