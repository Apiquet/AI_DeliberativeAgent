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
	TaskDistribution tdBackup;
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
		this.tdBackup = td;
		this.agent = agent;
		
		// initialize the planner
		int capacity = agent.vehicles().get(0).capacity();
		this.capacity = capacity;
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");
		
		// Throws IllegalArgumentException if algorithm is unknown
		this.algorithm = Algorithm.valueOf(algorithmName.toUpperCase());
		
		// ...
	}
	public void backupTD()
	{
		this.td=this.tdBackup;
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

	private Task IsThereAvalaibleTaskInCity(City city, Hashtable<Task,Double> task_table) {
		for(Entry<Task, Double> entry : task_table.entrySet()){
			if(entry.getKey().pickupCity == city){
		    	return entry.getKey();
		    }
		}
		return null;
	}
	private Plan BFSPlan(Vehicle vehicle, TaskSet tasks) {
		City currentCity = vehicle.getCurrentCity();
		Plan plan = new Plan(currentCity);
		HashMap<Plan,Double> plan_table=new HashMap<Plan, Double>();		
		Hashtable<Task,Double> task_table = new Hashtable<Task,Double>();
		State currentState = new State();
		int cost=0;
		int totalReward=0;
		int profit=0;
		int currentSpace = vehicle.capacity();
		for(int j=0;j<1;j++) {	

			for (Task task : tasks) {
				task_table.put(task, 1.0); //1 = task has to be taken, else 0
				System.out.println("Task= " + task.toString());

			}
			currentState = new State(currentCity, currentSpace, task_table);
			this.state_list.add(currentState);
	
			while(currentState.task_table.isEmpty() == false) {
				// move: current city => pickup location
				Random rand = new Random();
				City nextCity = currentCity.neighbors().get(rand.nextInt(currentCity.neighbors().size()));
				plan.appendMove(nextCity);
				cost+=currentCity.distanceTo(nextCity)*vehicle.costPerKm();
				currentCity = nextCity;
				Task task = IsThereAvalaibleTaskInCity(currentCity,task_table);
				if(task != null) {
					plan.appendPickup(task);
					for (City city :  currentCity.pathTo(task.deliveryCity)) {
						plan.appendMove(city);
						cost+=currentCity.distanceTo(city)*vehicle.costPerKm();
						currentCity = city;
					}
					plan.appendDelivery(task);
					currentSpace-=task.weight;
					// set current city
					currentCity = task.deliveryCity;
					totalReward += task.reward;
					task_table.remove(task);
				}
				currentState = new State(currentCity, currentSpace, task_table);
				this.state_list.add(currentState);
			}
			profit = totalReward-cost;
			plan_table.put(plan,(double) profit);
			System.out.println("Cost= " + cost);
			System.out.println("Reward= " + totalReward);
			System.out.println("Profit= " + profit);
			cost = 0;
			totalReward = 0;
			currentSpace = vehicle.capacity();
			backupTD();
			setup(this.topology, this.td, this.agent);

			for (City city :  currentCity.pathTo(this.topology.cities().get(0))) {
				plan.appendMove(city);
				currentCity = city;
			}
			
		}
		Plan bestPlan = null;
		double bestProfit = 0;

		for(Entry<Plan, Double> entry : plan_table.entrySet()){
			if(entry.getValue() > bestProfit){
				bestProfit= entry.getValue();
		    	bestPlan= entry.getKey();
		    }
			System.out.println("Best plan search= " + bestProfit);

		}
		if(bestPlan == null) throw new AssertionError("Best Plan not found.");

		System.out.println("Best profit= " + bestProfit);
		return bestPlan;
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