# File:         group_26 
# Description:  Improved solution to A* problem considering traffic evolution, without using memory
# Author:       Camilla, Esteban, Jordi, Assistant

library(DeliveryMan)

manhattan_distance <- function(a, b) {
  return(abs(a[1] - b[1]) + abs(a[2] - b[2]))
}


# Function to estimate future road cost
estimate_future_cost <- function(current_cost, steps_ahead) {
  # Probability of cost increase: 0.05
  # Probability of cost decrease: 0.05 (but only if cost > 1)
  # Maximum steps to look ahead
  max_steps <- 10
  steps <- min(steps_ahead, max_steps)
  
  estimated_cost <- current_cost
  for (i in 1:steps) {
    increase_prob <- 0.05
    decrease_prob <- if(estimated_cost > 1) 0.05 else 0
    
    estimated_cost <- estimated_cost + 
      (increase_prob - decrease_prob) # Expected value of cost change
  }
  
  return(max(1, estimated_cost)) # Ensure cost doesn't go below 1
}


astar <- function(start, goal, roads) {
  cat(sprintf("Starting A* algorithm. Start: (%d, %d), Goal: (%d, %d)\n", 
              start[1], start[2], goal[1], goal[2]))
  dim <- nrow(roads$hroads)
  
  # Initialize frontier as a list
  frontier <- list(list(pos = start, g = 0, h = manhattan_distance(start, goal), f = 0, path = list()))
  
  # Initialize closed set to keep track of explored nodes
  closed <- matrix(FALSE, nrow = dim, ncol = dim)
  
  # Initialize g_scores matrix to store the best known cost to reach each node
  g_scores <- matrix(Inf, nrow = dim, ncol = dim)
  g_scores[start[1], start[2]] <- 0
  
  iterations <- 0
  while (length(frontier) > 0) {
    iterations <- iterations + 1
    if (iterations %% 100 == 0) {
      cat(sprintf("A* iteration %d. Frontier size: %d\n", iterations, length(frontier)))
    }
    
    # Find the node with the lowest f-score in the frontier
    best_index <- which.min(sapply(frontier, function(node) node$f))
    current <- frontier[[best_index]]
    frontier <- frontier[-best_index]
    
    # Check if we've reached the goal
    if (all(current$pos == goal)) {
      cat(sprintf("A* found path to goal in %d iterations. Path length: %d\n", iterations, length(current$path)))
      return(current$path)
    }
    
    # Mark the current node as explored
    closed[current$pos[1], current$pos[2]] <- TRUE
    
    # Define and evaluate neighbors (up, down, right, left)
    neighbors <- list(
      list(pos = c(current$pos[1], current$pos[2] + 1), cost = roads$vroads[current$pos[1], current$pos[2]], move = 8),
      list(pos = c(current$pos[1], current$pos[2] - 1), cost = roads$vroads[current$pos[1], current$pos[2] - 1], move = 2),
      list(pos = c(current$pos[1] + 1, current$pos[2]), cost = roads$hroads[current$pos[1], current$pos[2]], move = 6),
      list(pos = c(current$pos[1] - 1, current$pos[2]), cost = roads$hroads[current$pos[1] - 1, current$pos[2]], move = 4)
    )
    
    for (neighbor in neighbors) {
      # Check if neighbor is within grid bounds
      if (neighbor$pos[1] < 1 || neighbor$pos[1] > dim || neighbor$pos[2] < 1 || neighbor$pos[2] > dim) {
        next
      }
      
      # Skip if this neighbor has already been explored
      if (closed[neighbor$pos[1], neighbor$pos[2]]) {
        next
      }
      
      # Not using estimate costs
      # g <- current$g + neighbor$cost
      
      # Estimate future cost based on current path length
      estimated_future_cost <- estimate_future_cost(neighbor$cost, length(current$path))
      
      # Calculate the cost to reach this neighbor through the current path
      g <- current$g + estimated_future_cost
      
      # If we've found a better path to this neighbor
      if (g < g_scores[neighbor$pos[1], neighbor$pos[2]]) {
        # Calculate heuristic cost (estimated cost from this neighbor to the goal)
        h <- manhattan_distance(neighbor$pos, goal)
        
        # Calculate total estimated cost (f = g + h)
        f <- g + h
        
        # Update the best known cost to reach this neighbor
        g_scores[neighbor$pos[1], neighbor$pos[2]] <- g
        
        # Create a new node for this neighbor
        new_node <- list(pos = neighbor$pos, g = g, h = h, f = f, path = c(current$path, neighbor$move))
        
        # Add the new node to the frontier
        frontier <- c(frontier, list(new_node))
      }
    }
  }
  
  # If we've exhausted all possible paths without reaching the goal
  cat(sprintf("WARNING: A* failed to find path after %d iterations\n", iterations))
  return(list())
}


find_closest_reachable_package <- function(car, packages, roads) {
  cat("- Starting find_closest_reachable_package function - \n")
  cat("- - - - - - - - - - -  \n")
  
  # Check if packages is valid
  if (is.null(packages) || nrow(packages) == 0 || ncol(packages) < 5) {
    cat("WARNING: No valid packages available\n")
    return(NULL)
  }
  
  # Initialize variables to keep track of the best package
  best_package <- NULL
  best_path <- NULL
  best_path_length <- Inf
  
  for (i in 1:nrow(packages)) {
    package <- packages[i, ]
    cat(sprintf("Evaluating package %d: Pickup (%d, %d), Delivery (%d, %d)\n", 
                i, package[1], package[2], package[3], package[4]))
    
    # Determine target based on whether car is empty or carrying a package
    target <- if (car$load == 0) c(package[1], package[2]) else c(package[3], package[4])
    
    # Calculate path using A* algorithm
    path <- astar(c(car$x, car$y), target, roads)
    
    if (length(path) > 0) {
      cat(sprintf("Found path to package %d. Path length: %d\n", i, length(path)))
      
      # Update best package if this one has a shorter path
      if (length(path) < best_path_length) {
        best_package <- package
        best_path <- path
        best_path_length <- length(path)
        cat(sprintf("New best package found: Package %d with path length %d\n", i, best_path_length))
      }
    } else {
      cat(sprintf("No path found to package %d\n", i))
    }
  }
  
  # If no reachable package found, use Manhattan distance as a fallback
  if (is.null(best_package)) {
    cat("No reachable packages found. Using Manhattan distance as fallback.\n")
    distances <- numeric(nrow(packages))
    for (i in 1:nrow(packages)) {
      package <- packages[i, ]
      current <- c(car$x, car$y)
      potential_package <- c(package[1], package[2])
      delivery_place <- c(package[3], package[4])
      pickup_distance <- manhattan_distance(current, potential_package)
      delivery_distance <- manhattan_distance(potential_package, delivery_place)
      distances[i] <- pickup_distance + delivery_distance
    }
    
    best_index <- which.min(distances)
    best_package <- packages[best_index, ]
    best_path_length <- distances[best_index]
  }
  
  if (!is.null(best_package)) {
    cat(sprintf("Closest package found. Pickup: (%d, %d), Delivery: (%d, %d), Path length/Distance: %.2f\n",
                best_package[1], best_package[2], best_package[3], best_package[4], best_path_length))
    return(list(package = best_package, path = best_path))
  } else {
    cat("WARNING: No packages found\n")
    return(NULL)
  }
}


myFunction <- function(roads, car, packages) {
  cat(sprintf("Starting new turn. Car position: (%d, %d), Load: %d\n", car$x, car$y, car$load))
  
  if (car$load == 0) {
    cat("Car has no package. Selecting best pickup.\n")
    available_packages <- if (!is.null(packages) && ncol(packages) >= 5) packages[packages[,5] == 0, , drop = FALSE] else NULL
    if (is.null(available_packages) || nrow(available_packages) == 0) {
      cat("No more packages to pick up. Game should end.\n")
      cat("\n")
      car$nextMove <- 5  # Stay still
      return(car)
    }
    # Changing conditions that's why we will be checking every move without load the best package to go after
    result <- find_closest_reachable_package(car, available_packages, roads)
    if (is.null(result)) {
      cat("WARNING: No reachable packages found. Keeps stucked.\n")
      cat("\n")
      car$nextMove <- 5
      return(car)
    }
    target <- result$package[1:2]
    path <- result$path
    cat(sprintf("Selected pickup target: (%d, %d)\n", target[1], target[2]))
  } else {
    cat("Car has a package. Going to delivery.\n")
    delivery <- if (!is.null(packages) && ncol(packages) >= 5) packages[packages[,5] == 1, , drop = FALSE] else NULL
    if (is.null(delivery) || nrow(delivery) == 0) {
      cat("ERROR: Car has a package but no delivery is scheduled. Staying still.\n")
      cat("\n")
      car$nextMove <- 5
      return(car)
    }
    target <- if (is.vector(delivery)) delivery[3:4] else delivery[1, 3:4]
    result <- astar(c(car$x, car$y), target, roads)  #find_closest_reachable_package(car, delivery, roads)
    if (is.null(result)) {
      cat("WARNING: Delivery unreachable. Keeps stucked.\n")
      cat("\n")
      car$nextMove <- 5
      return(car)
    }
    path <- result
    cat(sprintf("Delivery target: (%d, %d)\n", target[1], target[2]))
  }
  
  if (length(path) > 0) {
    car$nextMove <- path[[1]]
    cat(sprintf("Path found. Next move: %d\n", car$nextMove))
  } else {
    # If no path found, move towards the target using Manhattan distance
    #dx <- target[1] - car$x
    #dy <- target[2] - car$y
    #if (abs(dx) > abs(dy)) {
    #  car$nextMove <- if (dx > 0) 6 else 4
    #} else {
    #  car$nextMove <- if (dy > 0) 8 else 2
    #}
    #cat(sprintf("No path found. Moving towards target. Next move: %d\n", car$nextMove))
    cat("Path not found by A*. Not moving \n")
    car$nextMove <- 5
  }
  
  cat(sprintf("Turn completed. Next move: %d\n\n", car$nextMove))
  cat("---------------------------- \n")
  return(car)
}


# Run the game
runDeliveryMan(myFunction, doPlot = TRUE)