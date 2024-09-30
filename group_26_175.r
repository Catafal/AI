# File:         group_26 
# Description:  Improved solution to A* problem considering traffic evolution, without using memory
# Author:       Camilla, Esteban, Jordi

library(DeliveryMan)

# Calculate Manhattan distance between two points
manhattan_distance <- function(a, b) {
  return(abs(a[1] - b[1]) + abs(a[2] - b[2]))
}

# A* algorithm implementation
astar <- function(start, goal, roads) {
  dim <- nrow(roads$hroads)
  
  # Initialize frontier with start node
  frontier <- list(list(pos = start, g = 0, h = manhattan_distance(start, goal), f = 0, path = integer()))
  closed <- matrix(FALSE, nrow = dim, ncol = dim)
  
  while (length(frontier) > 0) {
    # Sort frontier by f-value and select the best node
    frontier <- frontier[order(sapply(frontier, function(node) node$f))]
    current <- frontier[[1]]
    frontier <- frontier[-1]
    
    # Check if we've reached the goal
    if (all(current$pos == goal)) {
      return(current$path)
    }
    
    # Process current node if it's within bounds and not visited
    if (all(current$pos >= 1) && all(current$pos <= dim)) {
      if (closed[current$pos[1], current$pos[2]]) {
        next
      }
      closed[current$pos[1], current$pos[2]] <- TRUE
    } else {
      next  # Skip this node if it's out of bounds
    }
    
    # Generate neighboring nodes
    neighbors <- list()
    moves <- list(c(0, 1, 8), c(0, -1, 2), c(1, 0, 6), c(-1, 0, 4))
    for (move in moves) {
      new_pos <- c(current$pos[1] + move[1], current$pos[2] + move[2])
      if (all(new_pos >= 1) && all(new_pos <= dim)) {
        neighbors <- c(neighbors, list(list(pos = new_pos, move = move[3])))
      }
    }
    
    # Process each neighbor
    for (neighbor in neighbors) {
      if (any(neighbor$pos < 1) || any(neighbor$pos > dim)) {
        next
      }
      
      # Calculate cost to move to neighbor
      cost <- if (neighbor$move %in% c(4, 6)) {
        if (neighbor$pos[1] >= 1 && neighbor$pos[1] <= dim && 
            current$pos[1] >= 1 && current$pos[1] <= dim && 
            neighbor$pos[2] >= 1 && neighbor$pos[2] <= dim) {
          roads$hroads[min(neighbor$pos[1], current$pos[1]), neighbor$pos[2]]
        } else {
          Inf
        }
      } else {
        if (neighbor$pos[1] >= 1 && neighbor$pos[1] <= dim && 
            neighbor$pos[2] >= 1 && neighbor$pos[2] <= dim && 
            current$pos[2] >= 1 && current$pos[2] <= dim) {
          roads$vroads[neighbor$pos[1], min(neighbor$pos[2], current$pos[2])]
        } else {
          Inf
        }
      }
      
      # Skip if cost is infinite or node is already closed
      if (is.infinite(cost) || (all(neighbor$pos >= 1) && all(neighbor$pos <= dim) && closed[neighbor$pos[1], neighbor$pos[2]])) {
        next
      }
      
      # Calculate f, g, and h values for neighbor
      g <- current$g + cost
      h <- manhattan_distance(neighbor$pos, goal)
      f <- g + h
      
      # Add neighbor to frontier
      new_node <- list(pos = neighbor$pos, g = g, h = h, f = f, path = c(current$path, neighbor$move))
      frontier <- c(frontier, list(new_node))
    }
  }
  
  return(integer())  # Return empty integer vector if no path found
}

# Find the best package to pick up next
find_closest_reachable_package2 <- function(car, packages, roads) {
  if (is.null(packages) || nrow(packages) == 0 || ncol(packages) < 5) {
    return(NULL)
  }
  
  # Estimate cost of a path considering traffic
  estimate_path_cost <- function(start, end, roads) {
    distance <- manhattan_distance(start, end)
    avg_traffic <- mean(c(roads$hroads, roads$vroads))
    return(distance * avg_traffic)
  }
  
  # Calculate total cost for a given order of packages
  calculate_order_cost <- function(order, car_pos) {
    total_cost <- 0
    current_pos <- car_pos
    
    for (i in order) {
      package <- packages[i, ]
      total_cost <- total_cost + estimate_path_cost(current_pos, package[1:2], roads)  # Pickup cost
      total_cost <- total_cost + estimate_path_cost(package[1:2], package[3:4], roads)  # Delivery cost
      current_pos <- package[3:4]  # Update position to delivery point
    }
    
    return(total_cost)
  }
  
  # Generate all possible orderings of packages
  n_packages <- nrow(packages)
  all_orders <- permutations(n_packages)
  
  # Calculate costs for all orderings
  order_costs <- sapply(1:nrow(all_orders), function(i) {
    calculate_order_cost(all_orders[i,], c(car$x, car$y))
  })
  
  # Find the best ordering
  best_order_index <- which.min(order_costs)
  best_order <- all_orders[best_order_index,]
  
  # Select the first package in the best order
  best_package <- packages[best_order[1], ]
  
  # Calculate the path to the best package
  best_path <- astar(c(car$x, car$y), best_package[1:2], roads)
  
  return(list(package = best_package, path = best_path))
}

# Generate all permutations of n elements
permutations <- function(n) {
  if (n == 1) {
    return(matrix(1))
  } else {
    sp <- permutations(n - 1)
    p <- nrow(sp)
    A <- matrix(nrow = n * p, ncol = n)
    for (i in 1:n) {
      A[(i-1)*p + 1:p, ] <- cbind(i, sp + (sp >= i))
    }
    return(A)
  }
}

# Main control function for the delivery man
myFunction <- function(roads, car, packages) {
  packages_to_deliver <- sum(packages[, 5] < 2)
  #cat(sprintf("Turn start: %d packages left to deliver.\n", packages_to_deliver))
  
  if (car$load == 0) {
    #cat("I am not loaded \n")
    available_packages <- packages[packages[,5] == 0, , drop = FALSE]
    if (nrow(available_packages) == 0) {
      car$nextMove <- 5  # Stay still if no packages to pick up
      return(car)
    }
    result <- find_closest_reachable_package2(car, available_packages, roads)
    if (is.null(result)) {
      car$nextMove <- 5  # Stay still if no reachable package
      return(car)
    } else {
      target <- result$package[1:2]
      path <- result$path
    }
  } else {
    #cat("I am loaded \n")
    delivery <- packages[packages[,5] == 1, , drop = FALSE]
    if (nrow(delivery) == 0) {
      car$nextMove <- 5  # Stay still if no delivery scheduled
      return(car)
    }
    target <- delivery[1, 3:4]
    path <- astar(c(car$x, car$y), target, roads)
  }
  
  # Determine next move
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
#runDeliveryMan(myFunction, doPlot = TRUE)
results_vector = testDM(myFunction, returnVec = TRUE)
#print(results_vector)
#cat("Mean score:", mean(results_vector), "\n")
#cat("Standard deviation:", sd(results_vector), "\n")
#cat("Minimum score:", min(results_vector), "\n")
#cat("Maximum score:", max(results_vector), "\n")