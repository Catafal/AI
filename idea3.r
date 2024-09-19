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
  dim <- nrow(roads$hroads)
  
  frontier <- list(list(pos = start, g = 0, h = manhattan_distance(start, goal), f = 0, path = list()))
  closed <- matrix(FALSE, nrow = dim, ncol = dim)
  
  while (length(frontier) > 0) {
    current <- frontier[[which.min(sapply(frontier, function(node) node$f))]]
    frontier <- frontier[-which.min(sapply(frontier, function(node) node$f))]
    
    if (all(current$pos == goal)) {
      return(current$path)
    }
    
    # Check if current position is within bounds before marking as closed
    if (all(current$pos >= 1) && all(current$pos <= dim)) {
      closed[current$pos[1], current$pos[2]] <- TRUE
    }
    
    neighbors <- list(
      list(pos = c(current$pos[1], current$pos[2] + 1), cost = ifelse(current$pos[2] < dim, roads$vroads[current$pos[1], current$pos[2]], Inf), move = 8),
      list(pos = c(current$pos[1], current$pos[2] - 1), cost = ifelse(current$pos[2] > 1, roads$vroads[current$pos[1], current$pos[2] - 1], Inf), move = 2),
      list(pos = c(current$pos[1] + 1, current$pos[2]), cost = ifelse(current$pos[1] < dim, roads$hroads[current$pos[1], current$pos[2]], Inf), move = 6),
      list(pos = c(current$pos[1] - 1, current$pos[2]), cost = ifelse(current$pos[1] > 1, roads$hroads[current$pos[1] - 1, current$pos[2]], Inf), move = 4)
    )
    
    for (neighbor in neighbors) {
      if (any(neighbor$pos < 1) || any(neighbor$pos > dim) || neighbor$cost == Inf) {
        next
      }
      
      if (closed[neighbor$pos[1], neighbor$pos[2]]) {
        next
      }
      
      g <- current$g + neighbor$cost
      h <- manhattan_distance(neighbor$pos, goal)
      f <- g + h
      
      new_node <- list(pos = neighbor$pos, g = g, h = h, f = f, path = c(current$path, neighbor$move))
      frontier <- c(frontier, list(new_node))
    }
  }
  
  return(list())  # Return empty list if no path found
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

find_closest_reachable_package2 <- function(car, packages, roads) {
  if (is.null(packages) || nrow(packages) == 0 || ncol(packages) < 5) {
    return(NULL)
  }
  
  # If there's only one package, return it directly
  if (nrow(packages) == 1) {
    path <- astar(c(car$x, car$y), packages[1, 1:2], roads)
    return(list(package = packages[1, ], path = path))
  }
  
  # Calculate Manhattan distances to all packages
  distances <- apply(packages[, 1:2], 1, function(p) manhattan_distance(c(car$x, car$y), p))
  
  # Sort packages by Manhattan distance
  sorted_indices <- order(distances)
  
  # Check the closest 3 packages (or all if less than 3)
  num_to_check <- min(3, nrow(packages))
  
  for (i in 1:num_to_check) {
    package <- packages[sorted_indices[i], ]
    path <- astar(c(car$x, car$y), package[1:2], roads)
    
    if (length(path) > 0) {
      return(list(package = package, path = path))
    }
  }
  
  # If no path found, return the closest package by Manhattan distance
  closest_package <- packages[sorted_indices[1], ]
  return(list(package = closest_package, path = list()))
}


myFunction <- function(roads, car, packages) {
  packages_to_deliver <- sum(packages[, 5] < 2)
  cat(sprintf("Turn start: %d packages left to deliver\n", packages_to_deliver))
  
  if (car$load == 0) {
    available_packages <- packages[packages[,5] == 0, , drop = FALSE]
    if (nrow(available_packages) == 0) {
      car$nextMove <- 5  # Stay still if no packages to pick up
      return(car)
    }
    result <- find_closest_reachable_package2(car, available_packages, roads)
    if (is.null(result)) {
      # Fallback: Move towards the nearest package using Manhattan distance
      # distances <- apply(available_packages[, 1:2], 1, function(p) manhattan_distance(c(car$x, car$y), p))
      # nearest_package <- available_packages[which.min(distances), ]
      # target <- nearest_package[1:2]
      
      car$nextMove <- 5  # Stay still if no reachable package
      return(car)
      
    } #else {
      target <- result$package[1:2]
      path <- result$path
    #}
  } else {
    delivery <- packages[packages[,5] == 1, , drop = FALSE]
    if (nrow(delivery) == 0) {
      car$nextMove <- 5  # Stay still if no delivery scheduled
      return(car)
    }
    target <- delivery[1, 3:4]
    path <- astar(c(car$x, car$y), target, roads)
  }
  
  #path <- astar(c(car$x, car$y), target, roads)
  
  if (length(path) > 0) {
    car$nextMove <- path[[1]]
  } else {
    # Fallback: Move towards the target using Manhattan distance
    dx <- target[1] - car$x
    dy <- target[2] - car$y
    if (abs(dx) > abs(dy)) {
      car$nextMove <- if (dx > 0) 6 else 4
    } else {
      car$nextMove <- if (dy > 0) 8 else 2
    }
  }
  
  return(car)
}


# Run the game
runDeliveryMan(myFunction, doPlot = TRUE)