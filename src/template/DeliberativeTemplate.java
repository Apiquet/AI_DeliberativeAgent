package template;

/* import table */
import logist.simulation.Vehicle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.concurrent.atomic.AtomicInteger;

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
public class DeliberativeTemplate implements DeliberativeBehavior {

	enum Algorithm { BFS, ASTAR }
	
	/* Environment */
	Topology topology;
	TaskDistribution td;
	ArrayList<State> finalstate_list= new ArrayList<State>();

	/* the properties of the agent */
	Agent agent;
	int capacity;

	/* the planning class */
	Algorithm algorithm;
	boolean debug = false;
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
		Plan plan = null;

		// Compute the plan with the selected algorithm.
		switch (algorithm) {
		case ASTAR:
			// ...
			try {
				plan = ASTARPlan(vehicle, tasks);
			} catch (CloneNotSupportedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			break;
		case BFS:
			// ...
			//plan = naivePlan(vehicle, tasks);
			try {
				plan = BFSPlan(vehicle, tasks);
			} catch (CloneNotSupportedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
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

	private ArrayList<Action> FindBestAction(HashMap<ArrayList<Action>,Double> action_table) {
		ArrayList<Action> bestActionList = null;
		double minCost = Double.MAX_VALUE;

		for(Entry<ArrayList<Action>, Double> entry : action_table.entrySet()){
			if(entry.getValue() < minCost){
				minCost= entry.getValue();
				bestActionList= entry.getKey();
		    }
		}
		System.out.println("Min Cost= " + minCost);
		return bestActionList;
	}
	private ArrayList<Action> FindBestState(ArrayList<State> state_list) {
		ArrayList<Action> bestActionList = null;
		double minCost = Double.MAX_VALUE;

		for(int i=0;i<state_list.size();i++){
			if(state_list.get(i).getCost() < minCost){
				minCost = state_list.get(i).getCost();	
				bestActionList= state_list.get(i).getActionList();
		    }
		}
		System.out.println("Min Cost= " + minCost);
		return bestActionList;
	}
	private Plan BuildingPlan(City initialCity, Plan plan, ArrayList<Action> action_list) {
		City currentCity = initialCity;
		for(int i=0;i<action_list.size();i++) {
			if(action_list.get(i).getCity()!=null) {
				//System.out.println("Move from= " + currentCity + bestActionList.get(i).getCity().toString());	
				currentCity = action_list.get(i).getCity();
				plan.appendMove(action_list.get(i).getCity());			
			}
			else {
				if(action_list.get(i).getPickup()) {
					//System.out.println("Pickup= " + bestActionList.get(i).getTask());
					plan.appendPickup(action_list.get(i).getTask());
				}
				else {					
					//System.out.println("deliver= " + bestActionList.get(i).getTask());
					plan.appendDelivery(action_list.get(i).getTask());
				}
			}
		}
		return plan;
	}
	
	private void RemovingSimilarStates(ArrayList<State> state_list) {
		System.out.println("RemovingSimilarState, init state_list_size: " + state_list.size());
		ArrayList<Integer> indexToDel = new ArrayList<Integer>();
		int i = 0;
		int j = 0;
		while(i<state_list.size()) {
			boolean i_removed=false;
			//System.out.println("state size: " + state_list.size() + "i= " + i);
			while(j<state_list.size()) {
				boolean j_removed=false;
				if(AreTaskTablesEqual(state_list.get(i).getTaskTable(),state_list.get(j).getTaskTable()) && i!=j && state_list.get(i).getCurrentCity() == state_list.get(j).getCurrentCity()) {
					if(state_list.get(i).getCost() > state_list.get(j).getCost()) {
						state_list.remove(i);
						//System.out.println("remove: " + i);
						i_removed = true;
						break;
					}
					else{
						state_list.remove(j);
						//System.out.println("remove: " + j);
						j_removed = false;
					}					
				}
				if(!j_removed) j++;
			}
			if(!i_removed) i++;
		}
		System.out.println("RemovingSimilarState, final state_list_size: " + state_list.size());
	}
	
	private boolean AreTaskTablesEqual(Hashtable<Task,Boolean> task_table1, Hashtable<Task,Boolean> task_table2) {
		if(task_table1.size() == task_table2.size()) {
			for (Entry<Task, Boolean> entry1 : task_table1.entrySet()) {
				for (Entry<Task, Boolean> entry2 : task_table2.entrySet()) {
					if(entry1.getKey() == entry2.getKey() && entry1.getValue()!=entry2.getValue()) return false;
				}
			}
		}
		
		return true;
	}
	
	private ArrayList<State> RemovingStateWithHigherCost(ArrayList<State> state_list, double cost) {
		System.out.println("RemovingStateWithHigherCost, init state_list_size: " + state_list.size());
		ArrayList<State> new_list = new ArrayList<State>();
		for(int i=0;i<state_list.size();i++) {
			if(state_list.get(i).getCost()<cost) {
				new_list.add(state_list.get(i));
			}
		}
		System.out.println("RemovingStateWithHigherCost, final state_list_size: " + new_list.size());
		return new_list;
	}
	
	private int calculNaiveCost(Hashtable<Task,Boolean> task_table, Vehicle vehicle) {
		int cost = 0;
		City currentCity = vehicle.getCurrentCity();
		City finalCity = vehicle.getCurrentCity();
		for (Entry<Task, Boolean> entry : task_table.entrySet()) {
			// move: current city => pickup location
			for (City city : currentCity.pathTo(entry.getKey().pickupCity)) {
				cost+=(int)finalCity.distanceTo(city)*vehicle.costPerKm();
				finalCity = city;
			}
			currentCity = entry.getKey().pickupCity;
			// move: pickup location => delivery location
			for (City city : currentCity.pathTo(entry.getKey().deliveryCity))
			{
				cost+=(int)finalCity.distanceTo(city)*vehicle.costPerKm();
				finalCity = city;
			}

			// set current city
			currentCity = entry.getKey().deliveryCity;
		}
		System.out.println("Naive cost= " + cost);

		return cost;
	}
	private Plan BFSPlan(Vehicle vehicle, TaskSet tasks) throws CloneNotSupportedException {			
		//Variables
		City currentCity = vehicle.getCurrentCity();
		ArrayList<Action> action_list = new ArrayList<Action>();
		ArrayList<State> state_list = new ArrayList<State>();
		Hashtable<Task,Boolean> task_table = new Hashtable<Task,Boolean>();
		State currentState = new State();		
		AtomicInteger currentSpace = new AtomicInteger(vehicle.capacity());
		int naiveCost = 0;
		//fetching all the tasks
		for (Task task : tasks) {
			task_table.put(task, true);
		}
		int task_number = task_table.size();
		naiveCost = calculNaiveCost(task_table, vehicle);
		//Declaring initial state with initial city, vehicle space and all the tasks
		currentState = new State(currentCity, currentSpace.get(), task_table, action_list, 0);
		state_list.add(currentState);
		int state_number = 0;
		int count=0;
		//building the plan until there is no more task to pick up or to deliver
		while(!state_list.isEmpty()) {
			state_number=state_list.size();
			for(int i=0;i<state_number;i++) {	
				for (Entry<Task, Boolean> entry : state_list.get(i).task_table.entrySet()) {
					State newState = state_list.get(i).clone();
					City final_City = newState.getCurrentCity();
					int cost = 0;
					if(entry.getValue() == true && entry.getKey().weight < newState.getCurrentSpace()) {
						// move: current city => pickup location
						for (City city : newState.getCurrentCity().pathTo(entry.getKey().pickupCity)) {
							newState.action_list.add(new Action(false,false,null,true,city));
							newState.increaseCost((int)final_City.distanceTo(city)*vehicle.costPerKm());
							final_City = city;
						}
						newState.action_list.add(new Action(true,false,entry.getKey(),true,null));
						newState.dicreaseCurrentSpace(entry.getKey().weight);
						newState.setCurrentCity(final_City);
						newState.updatingTaskTable(entry.getKey(), false);
					}
					else if(entry.getValue() == false) {

						// move: pickup location => delivery location
						for (City city : newState.getCurrentCity().pathTo(entry.getKey().deliveryCity)) {
							newState.action_list.add(new Action(false,false,null,true,city));
							newState.increaseCost((int)final_City.distanceTo(city)*vehicle.costPerKm());
							final_City = city;
						}
						newState.action_list.add(new Action(false,true,entry.getKey(),true,null));
						newState.setCurrentCity(final_City);
						newState.increaseCurrentSpace(entry.getKey().weight);
						newState.task_table.remove(entry.getKey());
					}
					else continue;
					if(newState.task_table.isEmpty()) {
						finalstate_list.add(newState);
					}
					else {
						state_list.add(newState);
					}						
				}	
			}
			for(int i=0;i<state_number;i++) {
				state_list.remove(0);
			}
			System.out.println("State number= " + state_list.size());
			//RemovingSimilarStates(state_list);
			count++;
			double progress = (double)count*10 /(task_number*2);
			System.out.println("Count= " + count);
			System.out.println("Progress= " + progress);
			if (count< 8) RemovingSimilarStates(state_list);
			//if (task_number> 5) state_list = RemovingStateWithHigherCost(state_list,naiveCost*0.45);
			if(progress>=3 && task_number>5) state_list = RemovingStateWithHigherCost(state_list,naiveCost*0.048*progress);
		}		
		//finding best action
		ArrayList<Action> bestActionList = FindBestState(finalstate_list);
		Plan plan = new Plan(vehicle.getCurrentCity());
		//building best plan with best action list
		plan=BuildingPlan(vehicle.getCurrentCity(), plan, bestActionList);		
		return plan;
	}
	
	private Plan ASTARPlan(Vehicle vehicle, TaskSet tasks) throws CloneNotSupportedException {			
		//Variables
		City currentCity = vehicle.getCurrentCity();
		ArrayList<Action> action_list = new ArrayList<Action>();
		ArrayList<State> state_list = new ArrayList<State>();
		ArrayList<State> state_list_prediction = new ArrayList<State>();
		Hashtable<Task,Boolean> task_table = new Hashtable<Task,Boolean>();
		State currentState = new State();		
		AtomicInteger currentSpace = new AtomicInteger(vehicle.capacity());
		int naiveCost = 0;
		//fetching all the tasks
		for (Task task : tasks) {
			task_table.put(task, true);
		}
		int task_number = task_table.size();
		naiveCost = calculNaiveCost(task_table, vehicle);
		//Declaring initial state with initial city, vehicle space and all the tasks
		currentState = new State(currentCity, currentSpace.get(), task_table, action_list, 0, 0);
		state_list.add(currentState);
		int state_number = 0;
		int count=0;
		double alpha = 2;
		//building the plan until there is no more task to pick up or to deliver
		while(!state_list.isEmpty()) {
			System.out.println("State number while beginning= " + state_list.size());

				int state_number_prediction = 0;
				state_number=state_list.size();
				for(int i=0;i<state_number;i++) {	
					for (Entry<Task, Boolean> entry : state_list.get(i).task_table.entrySet()) {
						State newState = state_list.get(i).clone();
						City final_City = newState.getCurrentCity();
						int cost = 0;
						if(entry.getValue() == true && entry.getKey().weight < newState.getCurrentSpace()) {
							// move: current city => pickup location
							for (City city : newState.getCurrentCity().pathTo(entry.getKey().pickupCity)) {
								newState.action_list.add(new Action(false,false,null,true,city));
								newState.increaseCost((int)final_City.distanceTo(city)*vehicle.costPerKm());
								final_City = city;
							}
							newState.action_list.add(new Action(true,false,entry.getKey(),true,null));
							newState.dicreaseCurrentSpace(entry.getKey().weight);
							newState.setCurrentCity(final_City);
							newState.updatingTaskTable(entry.getKey(), false);
						}
						else if(entry.getValue() == false) {
	
							// move: pickup location => delivery location
							for (City city : newState.getCurrentCity().pathTo(entry.getKey().deliveryCity)) {
								newState.action_list.add(new Action(false,false,null,true,city));
								newState.increaseCost((int)final_City.distanceTo(city)*vehicle.costPerKm());
								final_City = city;
							}
							newState.action_list.add(new Action(false,true,entry.getKey(),true,null));
							newState.setCurrentCity(final_City);
							newState.increaseCurrentSpace(entry.getKey().weight);
							newState.task_table.remove(entry.getKey());
						}
						else continue;
						if(newState.task_table.isEmpty()) {
							finalstate_list.add(newState);
						}
						else {
							state_list.add(newState);
							state_number_prediction ++;
						}						
					}
					System.out.println("State number prediction= " + state_list.size());
				}
				for(int i=0;i<state_number;i++) {
					System.out.println("Removing previous state");
					state_list.remove(0);				
				}

				System.out.println("Starting prediction, state_liste size= " + state_list.size());
				System.out.println("state_number_prediction= " + state_number_prediction);
				for(int i=state_number;i<state_number_prediction;i++) {
					int currentCost = Integer.MAX_VALUE;
					for (Entry<Task, Boolean> entry : state_list.get(i).task_table.entrySet()) {
						int cost_prediction = 0;
						State newState = state_list.get(i).clone();
						City final_City = newState.getCurrentCity();
						int cost = 0;
						if(entry.getValue() == true && entry.getKey().weight < newState.getCurrentSpace()) {
							// move: current city => pickup location
							for (City city : newState.getCurrentCity().pathTo(entry.getKey().pickupCity)) {
								newState.action_list.add(new Action(false,false,null,true,city));
								cost_prediction+=((int)final_City.distanceTo(city)*vehicle.costPerKm());
								final_City = city;
							}
							newState.action_list.add(new Action(true,false,entry.getKey(),true,null));
							newState.dicreaseCurrentSpace(entry.getKey().weight);
							newState.setCurrentCity(final_City);
							newState.updatingTaskTable(entry.getKey(), false);
						}
						else if(entry.getValue() == false) {
	
							// move: pickup location => delivery location
							for (City city : newState.getCurrentCity().pathTo(entry.getKey().deliveryCity)) {
								newState.action_list.add(new Action(false,false,null,true,city));
								cost_prediction+=((int)final_City.distanceTo(city)*vehicle.costPerKm());
								final_City = city;
							}
							newState.action_list.add(new Action(false,true,entry.getKey(),true,null));
							newState.setCurrentCity(final_City);
							newState.increaseCurrentSpace(entry.getKey().weight);
							newState.task_table.remove(entry.getKey());
						}
						else continue;
						if(currentCost > cost_prediction) {
							state_list.get(i).setAStarWeight(cost_prediction);	
							currentCost = cost_prediction;
						}
					}
				}
					
			for(int i=0;i<state_number_prediction-1;i++) {
				System.out.println("For loop Removing= " + i + " state number = " + state_list.size());
				System.out.println("state_number_prediction = " + state_number_prediction);
					if(state_list.get(i).aStarWeight > state_list.get(i+1).aStarWeight) {
						System.out.println("Removing= " + i);
						state_list.remove(i);
						i --;
					}
					else {
						System.out.println("Else Removing= " + i);
						state_list.remove(i+1);
					}
					state_number_prediction --;
				}				
			
			System.out.println("State number end after for= " + state_list.size());
			//RemovingSimilarStates(state_list);
			count++;
			double progress = (double)count*10 /(task_number*2);
			System.out.println("Count= " + count);
			System.out.println("Progress= " + progress);
			//if (count< 8) RemovingSimilarStates(state_list);
			//if (task_number> 5) state_list = RemovingStateWithHigherCost(state_list,naiveCost*0.45);
			//if(progress>=3 && task_number>5) state_list = RemovingStateWithHigherCost(state_list,naiveCost*0.048*progress);
		}		
		//finding best action
		ArrayList<Action> bestActionList = FindBestState(finalstate_list);
		Plan plan = new Plan(vehicle.getCurrentCity());
		//building best plan with best action list
		plan=BuildingPlan(vehicle.getCurrentCity(), plan, bestActionList);		
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
	/*
	 private ArrayList<State> RemovingStatesWithHigherCost(ArrayList<State> state_list) {
		int cost = state_list.get(0).getCost();
		ArrayList<State> new_list = new ArrayList<State>();
		State state = state_list.get(0);
		for(int i=0;i<state_list.size();i++) {
			if(state_list.get(i).getCost()<cost) {
				state = state_list.get(i);
			}
		}
		new_list.add(state);
		return new_list;
	}
	private boolean IsStatePresent(ArrayList<State> state_list, State state) {
		//System.out.println("New state= : " + state.toString());
		//System.out.println("list size= : " + state_list.size());
		
		for(int i=0;i<state_list.size();i++) {
			//System.out.println("Comparison state= : " + state_list.get(i).toString());
			if(state_list.get(i).getTaskTable().size() == state.getTaskTable().size() && AreTaskTablesEqual(state_list.get(i).getTaskTable(),state.getTaskTable())) {
				//System.out.println("EQUAL");
				return true;
			}
		}
		//System.out.println("NOT equal");

		return false;
	}
	private boolean filterStateList(ArrayList<State> state_list) {
		int cost = Integer.MAX_VALUE;
		for(int i=0;i<state_list.size();i++) {
			if(state_list.get(i).getCost()<cost) cost=state_list.get(i).getCost();
			else state_list.remove(i);
		}
		int emptyState = 0;
		for(int i=0;i<state_list.size();i++) {
			if(state_list.get(i).getTaskTable().isEmpty()) emptyState++;;
		}
		if(state_list.size() == emptyState) return false;
		return true;
	}
	private boolean isStateListTasksNotEmpty(ArrayList<State> state_list) {
		boolean bool = false;
		for(int i=0;i<state_list.size();i++) {
			if(!state_list.get(i).getTaskTable().isEmpty()) bool=true;
		}
		return bool;
	}
	private void PickUpTask(City city, Hashtable<Task,Double> task_table,ArrayList<Action> action_list, AtomicInteger currentSpace) {
		ArrayList<Task> tasksToPickUp = new ArrayList<Task>();
		for(Entry<Task, Double> entry : task_table.entrySet()){
			if(entry.getKey().pickupCity == city && entry.getValue() == 1 && currentSpace.get() > entry.getKey().weight){
				tasksToPickUp.add(entry.getKey());
				entry.setValue((double) 0);
				//Adding pickup action to action_list
				action_list.add(new Action(true,false,entry.getKey(),false,null));
				currentSpace.set(currentSpace.get()-entry.getKey().weight);
		    }
		}
	}
	private void DeliverTask(City city, Hashtable<Task,Double> task_table, ArrayList<Action> action_list, AtomicInteger currentSpace, AtomicInteger totalReward) {
		ArrayList<Task> tasksToDeliver = new ArrayList<Task>();
		for(Entry<Task, Double> entry : task_table.entrySet()){
			if(entry.getKey().deliveryCity == city && entry.getValue() == 0){
				tasksToDeliver.add(entry.getKey());
		    }
		}
		if(tasksToDeliver.size() != 0) {
			for(int i=0;i<tasksToDeliver.size();i++) {
				//Adding delivery action to action_list
				action_list.add(new Action(false,true,tasksToDeliver.get(i),false,null));
				currentSpace.set(currentSpace.get()+tasksToDeliver.get(i).weight);
				//updating task_table
				task_table.remove(tasksToDeliver.get(i));
				totalReward.set((int) (totalReward.get()+tasksToDeliver.get(i).reward));
			}				
		}			
	}
	private City MovingToRandomCity(ArrayList<Action> action_list, AtomicInteger cost, City currentCity, Vehicle vehicle) {
		// moving randomly:
		Random rand = new Random();
		City nextCity = currentCity.neighbors().get(rand.nextInt(currentCity.neighbors().size()));
		//adding move action to the action_list
		action_list.add(new Action(false,false,null,true,nextCity));
		//updating cost of the current plan
		cost.set((int) (cost.get()+currentCity.distanceTo(nextCity)*vehicle.costPerKm()));
		
		//updating current city
		currentCity = nextCity;	
		return currentCity;
	}

	 private Plan BFSRandPlan(Vehicle vehicle, TaskSet tasks) {
		
		//Variables
		City currentCity = vehicle.getCurrentCity();
		Plan plan = new Plan(currentCity);
		HashMap<Plan,Double> plan_table=new HashMap<Plan, Double>();
		HashMap<ArrayList<Action>,Double> action_table=new HashMap<ArrayList<Action>, Double>();	
		ArrayList<Action> action_list = new ArrayList<Action>();
		Hashtable<Task,Double> task_pickedUp = new Hashtable<Task,Double>();
		Hashtable<Task,Double> task_table = new Hashtable<Task,Double>();
		State currentState = new State();		
		Action currentAction = new Action();
		AtomicInteger cost = new AtomicInteger(0);
		AtomicInteger totalReward = new AtomicInteger(0);
		int profit=0; 
		AtomicInteger currentSpace = new AtomicInteger(vehicle.capacity());
		int numberOfPlanToTry = 10000;

		for(int j=0;j<numberOfPlanToTry;j++) {	
			//fetching all the tasks
			for (Task task : tasks) {
				task_table.put(task, 1.0);
			}
			
			//Declaring initial state with initial city, vehicle space and all the tasks
			currentState = new State(currentCity, currentSpace.get(), task_table);
			this.state_list.add(currentState);
	
			//building the plan until there is no more task to pick up or to deliver
			while(!currentState.task_table.isEmpty() || !task_pickedUp.isEmpty()) {
				
				// moving randomly:
				currentCity = MovingToRandomCity(action_list, cost, currentCity, vehicle);
				
				//verifying if there is any task to pick up in the current city
				PickUpTask(currentCity,task_table,task_pickedUp, action_list, currentSpace);				
				
				//verifying if there is any task to deliver in the current city
				DeliverTask(currentCity, task_pickedUp, action_list, currentSpace, totalReward);
				
				//updating current state and adding it to the state list
				currentState = new State(currentCity, currentSpace.get(), task_table);
				this.state_list.add(currentState);
			}
			
			//adding new plan found to a table
			action_table.put(action_list, (double) cost.get());

			//reset all the variables for the next plan discovery
			//currentCity = resetVariables(cost, totalReward, currentSpace, currentCity, action_list, task_pickedUp, vehicle);
			cost.set(0);
			totalReward.set(0);
			currentSpace.set(vehicle.capacity());	
			currentCity = vehicle.getCurrentCity();
			action_list = new ArrayList<Action>();
			task_pickedUp = new Hashtable<Task,Double>();			
		}
		
		//finding best action
		ArrayList<Action> bestActionList = FindBestAction(action_table);
		
		currentCity = vehicle.getCurrentCity();
		
		//building best plan with best action list
		plan=BuildingPlan(currentCity, plan, bestActionList);
		
		return plan;
	}*/
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
class State implements Cloneable {
	private City currentCity;
	private int currentSpace;
	Hashtable<Task,Boolean> task_table = new Hashtable<Task,Boolean>();
	ArrayList<Action> action_list = new ArrayList<Action>();
	int cost = 0;
	int aStarWeight = 0;
	public State clone() throws CloneNotSupportedException {
		State clonedObj = (State) super.clone();
        clonedObj.currentCity = this.currentCity;
        clonedObj.currentSpace = new Integer(this.currentSpace);
        clonedObj.cost = new Integer(this.cost);
        clonedObj.task_table = new Hashtable<Task,Boolean>(this.task_table);
        clonedObj.action_list = new ArrayList<Action>(this.action_list);
        return clonedObj;
    }
	public State(City currentcity, int currentSpace, Hashtable<Task,Boolean> task_table, ArrayList<Action> action_list, int cost) {				
		this.currentCity = currentcity;
		this.currentSpace = currentSpace;
		this.task_table = task_table;		
		this.action_list = action_list;		
		this.cost = cost;				
	}
	public State(City currentcity, int currentSpace, Hashtable<Task,Boolean> task_table, ArrayList<Action> action_list, int cost, int aStarWeight) {				
		this.currentCity = currentcity;
		this.currentSpace = currentSpace;
		this.task_table = task_table;		
		this.action_list = action_list;		
		this.cost = cost;			
		this.aStarWeight = aStarWeight;				
	}
	public String toString() {
		return "Current city= " + this.getCurrentCity().toString() +"Current space= " + this.getCurrentSpace() + "\n task table= " + this.getTaskTable().toString();

	}
	public State() {				
						
	}
	public City getCurrentCity() {
		return this.currentCity;
	}
	public void setCurrentCity(City city) {
		this.currentCity=city;
	}

	public int getCurrentSpace() {
		return this.currentSpace;
	}
	public int getCost() {
		return this.cost;
	}
	public void increaseCost(int cost) {
		this.cost += cost;
	}
	public void increaseAStarWeight(int aStarWeight) {
		this.aStarWeight += aStarWeight;
	}
	public void setAStarWeight(int aStarWeight) {
		this.aStarWeight = aStarWeight;
	}
	public void dicreaseCurrentSpace(int space) {
		this.currentSpace -= space;
	}
	public void increaseCurrentSpace(int space) {
		this.currentSpace += space;
	}

	public Hashtable<Task,Boolean> getTaskTable() {
		return this.task_table;
	}
	public void updatingTaskTable(Task task, boolean bool) {
		for (Entry<Task, Boolean> entry : this.task_table.entrySet()) {
			if(entry.getKey() == task) entry.setValue(bool);
		}
	}
	public ArrayList<Action> getActionList() {
		return this.action_list;
	}
}
