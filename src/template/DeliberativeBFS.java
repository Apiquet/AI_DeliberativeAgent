package template;

/* import table */
import logist.simulation.Vehicle;

import java.util.ArrayList;
import java.util.Hashtable;

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
	
	private Plan BFSPlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);
		ArrayList<Plan> plan_List= new ArrayList<Plan>();
		Hashtable<Task,Double> task_table = new Hashtable<Task,Double>();
		State currentState = new State();
		int cost=0;
		int totalReward=0;
		int profit=0;
		int currentSpace = vehicle.capacity();
		for (Task task : tasks) {
			task_table.put(task, 1.0); //1 = task has to be taken, else 0
		}
		this.state_list.add(new State(current, currentSpace, task_table));
		System.out.println("State list 1=" + this.state_list.toString());
		System.out.println("Task 1=" + task_table.toString());

		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity)) {
				plan.appendMove(city);
				cost+=current.distanceTo(city)*vehicle.costPerKm();
				current = city;
			}

			plan.appendPickup(task);
			currentSpace+=task.weight;
			
			// move: pickup location => delivery location
			for (City city : task.path()) {
				plan.appendMove(city);
				cost+=current.distanceTo(city)*vehicle.costPerKm();
				current = city;
			}

			plan.appendDelivery(task);
			currentSpace-=task.weight;

			// set current city
			current = task.deliveryCity;
			totalReward += task.reward;
			task_table.remove(task);
			currentState = new State(current, currentSpace, task_table);
			this.state_list.add(currentState);
			System.out.println("State List 2=" + this.state_list.toString());
			System.out.println("Task 2=" + task_table.toString());

		}
		//System.out.println("Plan Built");

		System.out.println("State List 3=" + this.state_list.toString());
		System.out.println("Task 3=" + task_table.toString());
		plan_List.add(plan);
		//System.out.println("Cost= " + cost);
		//System.out.println("Reward= " + totalReward);
		profit = totalReward-cost;
		//System.out.println("Profit= " + profit);
		cost = 0;
		totalReward = 0;
		currentSpace = 0;
		if(currentState.task_table.isEmpty()) System.out.println("Bingo");

		return plan_List.get(0);
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