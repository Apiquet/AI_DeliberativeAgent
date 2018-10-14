package template;

/* import table */
import logist.simulation.Vehicle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import javax.swing.JOptionPane;

import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Plan;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

/**
 * An optimal planner for one vehicle.
 */
@SuppressWarnings("unused")
public class DeliberativeBFS implements DeliberativeBehavior {

	enum Algorithm { BFS, ASTAR }
	
	/* Environment */
	Topology topology;
	TaskDistribution td;
	ArrayList<State> state_list= new ArrayList<State>();

	/* the properties of the agent */
	Agent agent;
	int capacity;

	/* the planning class */
	Algorithm algorithm;
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		this.topology = topology;
		this.td = td;
		this.agent = agent;
		
		// initialize the planner
		int capacity = agent.vehicles().get(0).capacity();
		this.capacity = capacity;
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");
		
		// Throws IllegalArgumentException if algorithm is unknown
		this.algorithm = Algorithm.valueOf(algorithmName.toUpperCase());
		
		// ...
	}
	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		Plan plan;

		// Compute the plan with the selected algorithm.
		switch (algorithm) {
		case ASTAR:
			// ...
			plan = naivePlan(vehicle, tasks);
			break;
		case BFS:
			// ...
			//plan = naivePlan(vehicle, tasks);
			plan = BFSPlan(vehicle, tasks);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
	}
	private Plan naivePlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity))
				plan.appendMove(city);

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path())
				plan.appendMove(city);

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}
		return plan;
	}

	private ArrayList<Task> IsThereAvalaibleTaskInCity(City city, Hashtable<Task,Double> task_table) {
		ArrayList<Task> tasksToPickUp = new ArrayList<Task>();
		for(Entry<Task, Double> entry : task_table.entrySet()){
			if(entry.getKey().pickupCity == city){
				tasksToPickUp.add(entry.getKey());
		    }
		}
		return tasksToPickUp;
	}
	private ArrayList<Task> IsThereTaskToDeliverInCity(City city, Hashtable<Task,Double> task_pickUp) {
		ArrayList<Task> tasksToDeliver = new ArrayList<Task>();
		for(Entry<Task, Double> entry : task_pickUp.entrySet()){
			if(entry.getKey().deliveryCity == city){
				tasksToDeliver.add(entry.getKey());
		    }
		}
		return tasksToDeliver;
	}
	private Plan BFSPlan(Vehicle vehicle, TaskSet tasks) {
		City currentCity = vehicle.getCurrentCity();
		Plan plan = new Plan(currentCity);
		HashMap<Plan,Double> plan_table=new HashMap<Plan, Double>();
		HashMap<ArrayList<Action>,Double> action_table=new HashMap<ArrayList<Action>, Double>();	
		ArrayList<Action> action_list = new ArrayList<Action>();
		Hashtable<Task,Double> task_pickedUp = new Hashtable<Task,Double>();
		Hashtable<Task,Double> task_table = new Hashtable<Task,Double>();
		State currentState = new State();		
		Action currentAction = new Action();
		int cost=0;
		int totalReward=0;
		int profit=0;
		int currentSpace = vehicle.capacity();
		
		//fetching all the tasks
		for(int j=0;j<5000;j++) {	
			for (Task task : tasks) {
				task_table.put(task, 1.0);
			}
			//Declaring initial state with initial city, vehicle space and all the tasks
			currentState = new State(currentCity, currentSpace, task_table);
			this.state_list.add(currentState);
	
			//building the plan until there is no more task to pick up or to deliver
			while(!currentState.task_table.isEmpty() || !task_pickedUp.isEmpty()) {
				// moving randomly:
				Random rand = new Random();
				City nextCity = currentCity.neighbors().get(rand.nextInt(currentCity.neighbors().size()));
				//adding move action to the action_list
				action_list.add(new Action(false,false,null,true,nextCity));
				//updating cost of the current plan
				cost+=currentCity.distanceTo(nextCity)*vehicle.costPerKm();
				//updating current city
				currentCity = nextCity;
				
				//verifying if there is any task to pick up in the current city
				ArrayList<Task> taskToPickUp = IsThereAvalaibleTaskInCity(currentCity,task_table);
				if(taskToPickUp.size() != 0) {
					for(int i=0;i<taskToPickUp.size();i++) {
						if(currentSpace > taskToPickUp.get(i).weight) {
							//plan.appendPickup(task);
							action_list.add(new Action(true,false,taskToPickUp.get(i),false,null));
							currentSpace-=taskToPickUp.get(i).weight;
							task_pickedUp.put(taskToPickUp.get(i),0.0);
							task_table.remove(taskToPickUp.get(i));
						}
					}
				}
				
				//verifying if there is any task to deliver in the current city
				ArrayList<Task> taskToDeliver = IsThereTaskToDeliverInCity(currentCity, task_pickedUp);
				if(taskToDeliver.size() != 0) {
					for(int i=0;i<taskToDeliver.size();i++) {
						//plan.appendPickup(task);
						action_list.add(new Action(false,true,taskToDeliver.get(i),false,null));
						currentSpace+=taskToDeliver.get(i).weight;
						task_pickedUp.remove(taskToDeliver.get(i));
						totalReward += taskToDeliver.get(i).reward;
					}					
				}	
				
				//updating current state and adding it to the state list
				currentState = new State(currentCity, currentSpace, task_table);
				this.state_list.add(currentState);
			}
			//calculating profit
			profit = totalReward-cost;
			//adding new plan found to a table
			action_table.put(action_list, (double) cost);

			//reset all the variables for the next plan discovery
			cost = 0;
			totalReward = 0;
			currentSpace = vehicle.capacity();	
			currentCity = vehicle.getCurrentCity();
			action_list = new ArrayList<Action>();
			task_pickedUp = new Hashtable<Task,Double>();
		}
		ArrayList<Action> bestActionList = null;
		double minCost = Double.MAX_VALUE;

		for(Entry<ArrayList<Action>, Double> entry : action_table.entrySet()){
			if(entry.getValue() < minCost){
				minCost= entry.getValue();
				bestActionList= entry.getKey();
		    }
		}
		currentCity = vehicle.getCurrentCity();
		for(int i=0;i<bestActionList.size();i++) {
			if(bestActionList.get(i).getCity()!=null) {
				//System.out.println("Move from= " + currentCity + bestActionList.get(i).getCity().toString());	
				currentCity = bestActionList.get(i).getCity();
				plan.appendMove(bestActionList.get(i).getCity());			
			}
			else {
				if(bestActionList.get(i).getPickup()) {
					//System.out.println("Pickup= " + bestActionList.get(i).getTask());
					plan.appendPickup(bestActionList.get(i).getTask());
				}
				else {					
					//System.out.println("deliver= " + bestActionList.get(i).getTask());
					plan.appendDelivery(bestActionList.get(i).getTask());
				}
			}
		}

		System.out.println("Min Cost= " + minCost);
		return plan;
	}

	@Override
	public void planCancelled(TaskSet carriedTasks) {
		
		if (!carriedTasks.isEmpty()) {
			// This cannot happen for this simple agent, but typically
			// you will need to consider the carriedTasks when the next
			// plan is computed.
		}
	}
}
class Action{
	boolean pickup=false;
	boolean delivery=false;
	Task task=null;
	boolean move=false;
	City city = null;
	public Action(boolean pickup,boolean delivery, Task task, boolean move, City city) {				
		this.pickup = pickup;
		this.delivery = delivery;
		this.task = task;
		this.move = move;		
		this.city = city;				
	}
	public Action() {					
	}
	public boolean getPickup() {
		return this.pickup;
	}
	public boolean getDelivery() {
		return this.delivery;
	}
	public boolean getMove() {
		return this.move;
	}
	public Task getTask() {
		return this.task;
	}
	public City getCity() {
		return this.city;
	}
}
class State {
	private City currentCity;
	private int availableSpace;
	Hashtable<Task,Double> task_table = new Hashtable<Task,Double>();
	public State(City currentcity, int availableSpace, Hashtable<Task,Double> task_table) {				
		this.currentCity = currentcity;
		this.availableSpace = availableSpace;
		this.task_table = task_table;				
	}
	public State() {				
						
	}
	public City getCurrentCity() {
		return this.currentCity;
	}

	public int getAvailableSpace() {
		return this.availableSpace;
	}
	
	public Hashtable<Task,Double> getTaskTable() {
		return this.task_table;
	}
}