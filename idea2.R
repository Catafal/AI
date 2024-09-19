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


manhattan_distance <- function(a, b) {
  return(abs(a[1] - b[1]) + abs(a[2] - b[2]))
}


astar <- function(start, goal, roads) {
  cat(sprintf("Starting A* algorithm. Start: (%d, %d), Goal: (%d, %d)\n", start[1], start[2], goal[1], goal[2]))
  dim <- nrow(roads$hroads)
  frontier <- list(list(pos = start, g = 0, h = manhattan_distance(start, goal), f = 0, path = list()))
  closed <- matrix(FALSE, nrow = dim, ncol = dim)
  
  iterations <- 0
  while (length(frontier) > 0) {
    iterations <- iterations + 1
    if (iterations %% 100 == 0) {
      cat(sprintf("A* iteration %d. Frontier size: %d\n", iterations, length(frontier)))
    }
    
    f_scores <- sapply(frontier, function(node) node$f)
    current <- frontier[[which.min(f_scores)]]
    frontier <- frontier[-which.min(f_scores)]
    
    if (all(current$pos == goal)) {
      cat(sprintf("A* found path to goal in %d iterations. Path length: %d\n", iterations, length(current$path)))
      return(current$path)
    }
    
    closed[current$pos[1], current$pos[2]] <- TRUE
    
    neighbors <- list(
      list(pos = c(current$pos[1], current$pos[2] + 1), cost = roads$vroads[current$pos[1], current$pos[2]], move = 8),
      list(pos = c(current$pos[1], current$pos[2] - 1), cost = roads$vroads[current$pos[1], current$pos[2] - 1], move = 2),
      list(pos = c(current$pos[1] + 1, current$pos[2]), cost = roads$hroads[current$pos[1], current$pos[2]], move = 6),
      list(pos = c(current$pos[1] - 1, current$pos[2]), cost = roads$hroads[current$pos[1] - 1, current$pos[2]], move = 4)
    )
    
    for (neighbor in neighbors) {
      if (neighbor$pos[1] < 1 || neighbor$pos[1] > dim || neighbor$pos[2] < 1 || neighbor$pos[2] > dim) {
        next
      }
      
      if (closed[neighbor$pos[1], neighbor$pos[2]]) {
        next
      }
      
      g <- current$g + neighbor$cost
      h <- manhattan_distance(neighbor$pos, goal)
      f <- g + h
      
      # Check if the neighbor is already in the frontier
      existing_index <- NULL
      for (i in seq_along(frontier)) {
        if (all(frontier[[i]]$pos == neighbor$pos)) {
          existing_index <- i
          break
        }
      }
      
      if (!is.null(existing_index)) {
        if (frontier[[existing_index]]$f <= f) {
          next
        } else {
          # If the new path is better, update the existing node
          frontier[[existing_index]] <- list(pos = neighbor$pos, g = g, h = h, f = f, path = c(current$path, neighbor$move))
        }
      } else {
        # If the node is not in the frontier, add it
        new_node <- list(pos = neighbor$pos, g = g, h = h, f = f, path = c(current$path, neighbor$move))
        frontier <- c(frontier, list(new_node))
      }
    }
  }
  
  cat(sprintf("WARNING: A* failed to find path after %d iterations\n", iterations))
  return(list())
}


select_best_package <- function(car, packages, roads) {
  cat(sprintf("Selecting best package. Available packages: %d\n", nrow(packages)))
  
  if (nrow(packages) == 0) {
    cat("WARNING: No packages available for pickup\n")
    return(NULL)
  }
  
  total_distances <- apply(packages, 1, function(package) {
    pickup_dist <- manhattan_distance(c(car$x, car$y), c(package[1], package[2]))
    delivery_dist <- manhattan_distance(c(package[1], package[2]), c(package[3], package[4]))
    total_dist <- pickup_dist + delivery_dist
    cat(sprintf("Package (%d, %d) -> (%d, %d). Total distance: %d\n", 
                package[1], package[2], package[3], package[4], total_dist))
    return(total_dist)
  })
  
  best_package <- packages[which.min(total_distances), ]
  cat(sprintf("Selected best package: Pickup (%d, %d), Delivery (%d, %d)\n", 
              best_package[1], best_package[2], best_package[3], best_package[4]))
  return(best_package)
}


find_closest_reachable_package <- function(car, packages, roads) {
  if (is.null(packages) || nrow(packages) == 0 || ncol(packages) == 0) {
    return(NULL)
  }
  for (i in 1:nrow(packages)) {
    package <- packages[i, ]
    target <- if (car$load == 0) c(package[1], package[2]) else c(package[3], package[4])
    path <- astar(c(car$x, car$y), target, roads)
    if (length(path) > 0) {
      return(list(package = package, path = path))
    }
  }
  return(NULL)
}


myFunction <- function(roads, car, packages) {
  cat(sprintf("Starting new turn. Car position: (%d, %d), Load: %d\n", car$x, car$y, car$load))
  
  if (car$load == 0) {
    cat("Car has no package. Selecting best pickup.\n")
    available_packages <- if (!is.null(packages) && ncol(packages) >= 5) packages[packages[,5] == 0, , drop = FALSE] else NULL
    if (is.null(available_packages) || nrow(available_packages) == 0) {
      cat("No more packages to pick up. Game should end.\n")
      car$nextMove <- 5  # Stay still
      return(car)
    }
    result <- find_closest_reachable_package(car, available_packages, roads)
    if (is.null(result)) {
      cat("WARNING: No reachable packages. Moving randomly.\n")
      car$nextMove <- 5 #sample(c(2,4,6,8), 1)
      return(car)
    }
    target <- result$package
    path <- result$path
    cat(sprintf("Selected pickup target: (%d, %d)\n", target[1], target[2]))
  } else {
    cat("Car has a package. Going to delivery.\n")
    delivery <- if (!is.null(packages) && ncol(packages) >= 5) packages[packages[,5] == 1, , drop = FALSE] else NULL
    if (is.null(delivery) || nrow(delivery) == 0) {
      cat("ERROR: Car has a package but no delivery is scheduled. Moving randomly.\n")
      car$nextMove <- sample(c(2,4,6,8), 1)
      return(car)
    }
    result <- find_closest_reachable_package(car, delivery, roads)
    if (is.null(result)) {
      cat("WARNING: Delivery unreachable. Moving randomly.\n")
      car$nextMove <- 5 #sample(c(2,4,6,8), 1)
      return(car)
    }
    target <- result$package
    path <- result$path
    cat(sprintf("Delivery target: (%d, %d)\n", target[3], target[4]))
  }
  
  if (length(path) > 0) {
    car$nextMove <- path[[1]]
    cat(sprintf("Path found. Next move: %d\n", car$nextMove))
  } else {
    cat("WARNING: No path found. Moving randomly.\n")
    car$nextMove <- 5 #sample(c(2,4,6,8), 1)
  }
  
  cat(sprintf("Turn completed. Next move: %d\n\n", car$nextMove))
  return(car)
}


runDeliveryMan(myFunction, doPlot = TRUE)

# testDM(myFunction)
