# File:         group_26 
# Description:  Solution to A* problem
# Author:       Camilla, Esteban, Jordi

# Install the package
# install.packages("DeliveryMan_1.1.0.tar.gz", repos = NULL, type="source")

# Load the library
library(DeliveryMan)

# Read documentation
# ?runDeliveryMan
# ?testDM


manhattan_distance <- function(x1, y1, x2, y2) {
  return(abs(x1 - x2) + abs(y1 - y2))
}

astar <- function(start_x, start_y, goal_x, goal_y, hroads, vroads, max_iterations = 1000, ignore_costs = FALSE) {
  dim_x <- ncol(vroads)
  dim_y <- nrow(hroads)
  open_list <- list()
  closed_set <- matrix(FALSE, nrow = dim_y, ncol = dim_x)
  
  start_node <- list(x = start_x, y = start_y, g = 0, 
                     h = manhattan_distance(start_x, start_y, goal_x, goal_y),
                     f = 0, parent = NULL)
  open_list[[paste(start_x, start_y)]] <- start_node
  
  cat(sprintf("A* Debug: Start (%d, %d), Goal (%d, %d), Ignore costs: %s\n", 
              start_x, start_y, goal_x, goal_y, ignore_costs))
  
  iteration <- 0
  while (length(open_list) > 0 && iteration < max_iterations) {
    iteration <- iteration + 1
    current_key <- names(which.min(sapply(open_list, function(node) node$f)))
    current <- open_list[[current_key]]
    
    cat(sprintf("A* Debug: Iteration %d, Current (%d, %d), f = %.2f\n", 
                iteration, current$x, current$y, current$f))
    
    if (current$x == goal_x && current$y == goal_y) {
      cat("A* Debug: Goal reached!\n")
      path <- list()
      while (!is.null(current$parent)) {
        path <- c(list(list(x = current$x, y = current$y)), path)
        current <- current$parent
      }
      return(list(path = path, complete = TRUE))
    }
    
    open_list[[current_key]] <- NULL
    closed_set[current$y, current$x] <- TRUE
    
    neighbors <- get_neighbors(current$x, current$y, hroads, vroads)
    cat(sprintf("A* Debug: Found %d neighbors\n", length(neighbors)))
    
    for (neighbor in neighbors) {
      if (neighbor$x < 1 || neighbor$x > dim_x || neighbor$y < 1 || neighbor$y > dim_y) {
        cat(sprintf("A* Debug: Neighbor (%d, %d) out of bounds\n", neighbor$x, neighbor$y))
        next
      }
      if (closed_set[neighbor$y, neighbor$x]) {
        cat(sprintf("A* Debug: Neighbor (%d, %d) already closed\n", neighbor$x, neighbor$y))
        next
      }
      
      tentative_g <- current$g + if (ignore_costs) 1 else neighbor$cost
      
      neighbor_key <- paste(neighbor$x, neighbor$y)
      if (is.null(open_list[[neighbor_key]]) || tentative_g < open_list[[neighbor_key]]$g) {
        open_list[[neighbor_key]] <- list(
          x = neighbor$x,
          y = neighbor$y,
          g = tentative_g,
          h = manhattan_distance(neighbor$x, neighbor$y, goal_x, goal_y),
          f = tentative_g + manhattan_distance(neighbor$x, neighbor$y, goal_x, goal_y),
          parent = current
        )
        cat(sprintf("A* Debug: Added/Updated neighbor (%d, %d), f = %.2f\n", 
                    neighbor$x, neighbor$y, open_list[[neighbor_key]]$f))
      }
    }
  }
  
  cat("A* Debug: No path found\n")
  return(list(path = NULL, complete = FALSE))
}


get_neighbors <- function(x, y, hroads, vroads) {
  neighbors <- list()
  dim_x <- ncol(vroads)
  dim_y <- nrow(hroads)
  
  cat(sprintf("Get Neighbors Debug: Checking neighbors for (%d, %d)\n", x, y))
  
  # Check right
  if (x < dim_x && y <= dim_y) {
    neighbors <- c(neighbors, list(list(x = x + 1, y = y, cost = hroads[y, x])))
    cat(sprintf("Get Neighbors Debug: Added right neighbor (%d, %d)\n", x + 1, y))
  }
  # Check left
  if (x > 1 && y <= dim_y) {
    neighbors <- c(neighbors, list(list(x = x - 1, y = y, cost = hroads[y, x - 1])))
    cat(sprintf("Get Neighbors Debug: Added left neighbor (%d, %d)\n", x - 1, y))
  }
  # Check up
  if (y < dim_y && x <= dim_x) {
    neighbors <- c(neighbors, list(list(x = x, y = y + 1, cost = vroads[y, x])))
    cat(sprintf("Get Neighbors Debug: Added up neighbor (%d, %d)\n", x, y + 1))
  }
  # Check down
  if (y > 1 && x <= dim_x) {
    neighbors <- c(neighbors, list(list(x = x, y = y - 1, cost = vroads[y - 1, x])))
    cat(sprintf("Get Neighbors Debug: Added down neighbor (%d, %d)\n", x, y - 1))
  }
  
  cat(sprintf("Get Neighbors Debug: Found %d neighbors\n", length(neighbors)))
  return(neighbors)
}


# Approach:
# - Compute the manhattan distance from Start Point to Pick Up Point and from Pick Up Point to Delivery Point. For all the packages.
# - Then sum both distances and choose which path to follow.
# - With the choosen path, only calculate each step the manhattan distance until the next step point. So if we do not have a 
#   package yet, we will be calculating the distance until Pick Up Point only. And once reach it, with the package,
#   we will be calculating the distance for the Delivery Point.

#Next approach, to select the path considering the costs. So using the a* instead of only manhattan. Or with a greedy.

myFunction <- function(roads, car, packages) {
  x <- car$x
  y <- car$y
  
  cat(sprintf("\nCurrent position: (%d, %d), Load: %d\n", x, y, car$load))
  
  # Check if we're at a delivery or pickup point
  at_delivery <- any(packages[, 3] == x & packages[, 4] == y & packages[, 5] == 1)
  at_pickup <- any(packages[, 1] == x & packages[, 2] == y & packages[, 5] == 0)
  
  if (at_delivery || at_pickup) {
    car$nextMove <- 5  # Stay still to ensure pickup/delivery
    cat("At delivery/pickup point. Staying still.\n")
    return(car)
  }
  
  if (car$load == 0) {
    cat("No package loaded. Looking for pickup...\n")
    undelivered <- packages[packages[, 5] == 0, , drop = FALSE]
    if (nrow(undelivered) > 0) {
      distances <- t(apply(undelivered, 1, function(pkg) {
        pickup_distance <- manhattan_distance(x, y, pkg[1], pkg[2])
        delivery_distance <- manhattan_distance(pkg[1], pkg[2], pkg[3], pkg[4])
        total_distance <- pickup_distance + delivery_distance
        return(c(total_distance, pickup_distance))
      }))
      
      best_package <- which.min(distances[, 1])
      
      if (sum(distances[, 1] == distances[best_package, 1]) > 1) {
        tied_packages <- which(distances[, 1] == distances[best_package, 1])
        best_package <- tied_packages[which.min(distances[tied_packages, 2])]
      }
      
      goal_x <- undelivered[best_package, 1]
      goal_y <- undelivered[best_package, 2]
      cat(sprintf("Chosen pickup point: (%d, %d)\n", goal_x, goal_y))
    } else {
      car$nextMove <- 5  # Stay still if no packages left
      cat("No undelivered packages left. Staying still.\n")
      return(car)
    }
  } else {
    cat("Package loaded. Looking for delivery point...\n")
    package <- packages[packages[, 5] == 1, , drop = FALSE]
    if (nrow(package) > 0) {
      goal_x <- package[1, 3]
      goal_y <- package[1, 4]
      cat(sprintf("Delivery point: (%d, %d)\n", goal_x, goal_y))
    } else {
      car$nextMove <- 5  # Stay still
      cat("No delivery point found. This shouldn't happen. Staying still.\n")
      return(car)
    }
  }
  
  cat("Calculating A* path with costs...\n")
  path_result <- astar(x, y, goal_x, goal_y, roads$hroads, roads$vroads)
  
  if (length(path_result$path) == 0) {
    cat("No path found with costs. Recalculating without costs...\n")
    path_result <- astar(x, y, goal_x, goal_y, roads$hroads, roads$vroads, ignore_costs = TRUE)
  }
  
  if (length(path_result$path) > 0) {
    next_pos <- path_result$path[[1]]
    cat(sprintf("Next position: (%d, %d)\n", next_pos$x, next_pos$y))
    if (next_pos$x > x) {
      car$nextMove <- 6      # East
      cat("Moving East\n")
    } else if (next_pos$x < x) {
      car$nextMove <- 4      # West
      cat("Moving West\n")
    } else if (next_pos$y > y) {
      car$nextMove <- 8      # North
      cat("Moving North\n")
    } else if (next_pos$y < y) {
      car$nextMove <- 2      # South
      cat("Moving South\n")
    } else {
      car$nextMove <- 5      # Stay still
      cat("Staying still\n")
    }
  } else {
    # If still no path, move towards the goal using Manhattan distance
    if (x < goal_x) {
      car$nextMove <- 6  # East
      cat("No path found. Moving East towards goal.\n")
    } else if (x > goal_x) {
      car$nextMove <- 4  # West
      cat("No path found. Moving West towards goal.\n")
    } else if (y < goal_y) {
      car$nextMove <- 8  # North
      cat("No path found. Moving North towards goal.\n")
    } else if (y > goal_y) {
      car$nextMove <- 2  # South
      cat("No path found. Moving South towards goal.\n")
    } else {
      car$nextMove <- 5  # Stay still
      cat("No path found. At goal position. Staying still.\n")
    }
  }
  
  cat(sprintf("Next move: %d\n", car$nextMove))
  return(car)
}



runDeliveryMan(myFunction)