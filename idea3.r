# File:         group_26 
# Description:  Improved solution to A* problem considering traffic evolution, without using memory
# Author:       Camilla, Esteban, Jordi

library(DeliveryMan)

manhattan_distance <- function(a, b) {
  return(abs(a[1] - b[1]) + abs(a[2] - b[2]))
}

astar <- function(start, goal, roads) {
  dim <- nrow(roads$hroads)
  
  frontier <- list(list(pos = start, g = 0, h = manhattan_distance(start, goal), f = 0, path = integer()))
  closed <- matrix(FALSE, nrow = dim, ncol = dim)
  
  while (length(frontier) > 0) {
    # Sort the frontier by f-value and select the first (best) node
    frontier <- frontier[order(sapply(frontier, function(node) node$f))]
    current <- frontier[[1]]
    frontier <- frontier[-1]
    
    if (all(current$pos == goal)) {
      return(current$path)
    }
    
    # Check if current position is within bounds
    if (all(current$pos >= 1) && all(current$pos <= dim)) {
      if (closed[current$pos[1], current$pos[2]]) {
        next
      }
      closed[current$pos[1], current$pos[2]] <- TRUE
    } else {
      next  # Skip this node if it's out of bounds
    }
    
    neighbors <- list(
      list(pos = c(current$pos[1], current$pos[2] + 1), move = 8),
      list(pos = c(current$pos[1], current$pos[2] - 1), move = 2),
      list(pos = c(current$pos[1] + 1, current$pos[2]), move = 6),
      list(pos = c(current$pos[1] - 1, current$pos[2]), move = 4)
    )
    
    for (neighbor in neighbors) {
      if (any(neighbor$pos < 1) || any(neighbor$pos > dim)) {
        next
      }
      
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
      
      if (is.infinite(cost) || (all(neighbor$pos >= 1) && all(neighbor$pos <= dim) && closed[neighbor$pos[1], neighbor$pos[2]])) {
        next
      }
      
      g <- current$g + cost
      h <- manhattan_distance(neighbor$pos, goal)
      f <- g + h
      
      new_node <- list(pos = neighbor$pos, g = g, h = h, f = f, path = c(current$path, neighbor$move))
      frontier <- c(frontier, list(new_node))
    }
  }
  
  return(integer())  # Return empty integer vector if no path found
}

find_closest_reachable_package <- function(car, packages, roads) {
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

find_closest_reachable_package2 <- function(car, packages, roads) {
  if (is.null(packages) || nrow(packages) == 0 || ncol(packages) < 5) {
    return(NULL)
  }
  
  # Helper function to calculate Manhattan distance
  manhattan_distance <- function(a, b) {
    return(abs(a[1] - b[1]) + abs(a[2] - b[2]))
  }
  
  # Function to estimate the cost of a path considering traffic
  estimate_path_cost <- function(start, end, roads) {
    distance <- manhattan_distance(start, end)
    avg_traffic <- mean(c(roads$hroads, roads$vroads))
    return(distance * avg_traffic)
  }
  
  # Function to calculate the total cost of a given order of packages
  calculate_order_cost <- function(order, car_pos) {
    total_cost <- 0
    current_pos <- car_pos
    
    for (i in order) {
      package <- packages[i, ]
      # Cost to pick up
      total_cost <- total_cost + estimate_path_cost(current_pos, package[1:2], roads)
      # Cost to deliver
      total_cost <- total_cost + estimate_path_cost(package[1:2], package[3:4], roads)
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

# Helper function to generate all permutations
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



myFunction <- function(roads, car, packages) {
  packages_to_deliver <- sum(packages[, 5] < 2)
  cat(sprintf("Turn start: %d packages left to deliver.\n", packages_to_deliver))
  
  if (car$load == 0) {
    cat("I am loaded \n")
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
    cat("I am NOT loaded \n")
    delivery <- packages[packages[,5] == 1, , drop = FALSE]
    if (nrow(delivery) == 0) {
      car$nextMove <- 5  # Stay still if no delivery scheduled
      return(car)
    }
    target <- delivery[1, 3:4]
    path <- astar(c(car$x, car$y), target, roads)
  }
  
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
print(results_vector)
cat("Mean score:", mean(results_vector), "\n")
#cat("Standard deviation:", sd(results_vector), "\n")
#cat("Minimum score:", min(results_vector), "\n")
#cat("Maximum score:", max(results_vector), "\n")
