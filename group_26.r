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

get_neighbors <- function(x, y, hroads, vroads) {
  neighbors <- list()
  dim <- nrow(hroads)  # Assuming square grid
  
  # Check left
  if (x > 1) {
    neighbors <- c(neighbors, list(list(x = x - 1, y = y, cost = hroads[x - 1, y])))
  }
  # Check right
  if (x < dim) {
    neighbors <- c(neighbors, list(list(x = x + 1, y = y, cost = hroads[x, y])))
  }
  # Check down
  if (y > 1) {
    neighbors <- c(neighbors, list(list(x = x, y = y - 1, cost = vroads[x, y - 1])))
  }
  # Check up
  if (y < dim) {
    neighbors <- c(neighbors, list(list(x = x, y = y + 1, cost = vroads[x, y])))
  }
  
  return(neighbors)
}

astar <- function(start_x, start_y, goal_x, goal_y, hroads, vroads) {
  open_list <- list()
  closed_set <- matrix(FALSE, nrow = nrow(hroads), ncol = ncol(hroads))
  
  start_node <- list(x = start_x, y = start_y, g = 0, 
                     h = manhattan_distance(start_x, start_y, goal_x, goal_y),
                     f = 0, parent = NULL)
  open_list[[paste(start_x, start_y)]] <- start_node
  
  while (length(open_list) > 0) {
    current_key <- names(which.min(sapply(open_list, function(node) node$f)))
    current <- open_list[[current_key]]
    
    if (current$x == goal_x && current$y == goal_y) {
      path <- list()
      while (!is.null(current$parent)) {
        path <- c(list(list(x = current$x, y = current$y)), path)
        current <- current$parent
      }
      return(path)
    }
    
    open_list[[current_key]] <- NULL
    closed_set[current$x, current$y] <- TRUE
    
    neighbors <- get_neighbors(current$x, current$y, hroads, vroads)
    
    for (neighbor in neighbors) {
      if (closed_set[neighbor$x, neighbor$y]) {
        next
      }
      
      tentative_g <- current$g + neighbor$cost
      
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
      }
    }
  }
  
  return(NULL)  # No path found
}


# Approach:
# - Compute the manhattan distance from Start Point to Pick Up Point and from Pick Up Point to Delivery Point. For all the packages.
# - Then sum both distances and choose which path to follow.
# - With the choosen path, only calculate each step the manhattan distance until the next step point. So if we do not have a 
#   package yet, we will be calculating the distance until Pick Up Point only. And once reach it, with the package,
#   we will be calculating the distance for the Delivery Point.

myFunction <- function(roads, car, packages) {
  x <- car$x
  y <- car$y
  
  if (car$load == 0) {
    undelivered <- packages[packages[, 5] == 0, , drop = FALSE]
    if (nrow(undelivered) > 0) {
      if (nrow(undelivered) == 1) {
        goal_x <- undelivered[1, 1]
        goal_y <- undelivered[1, 2]
      } else {
        distances <- t(apply(undelivered, 1, function(pkg) {
          pickup_distance <- manhattan_distance(x, y, pkg[1], pkg[2])
          delivery_distance <- manhattan_distance(pkg[1], pkg[2], pkg[3], pkg[4])
          total_distance <- pickup_distance + delivery_distance
          return(c(total_distance, pickup_distance))
        }))
        
        min_total_distance <- min(distances[, 1])
        min_distance_packages <- which(distances[, 1] == min_total_distance)
        
        if (length(min_distance_packages) > 1) {
          best_package <- min_distance_packages[which.min(distances[min_distance_packages, 2])]
        } else {
          best_package <- min_distance_packages
        }
        
        goal_x <- undelivered[best_package, 1]
        goal_y <- undelivered[best_package, 2]
      }
    } else {
      car$nextMove <- 5  # Stay still
      return(car)
    }
  } else {
    package <- packages[packages[, 5] == 1, , drop = FALSE]
    if (nrow(package) > 0) {
      goal_x <- package[1, 3]
      goal_y <- package[1, 4]
    } else {
      car$nextMove <- 5  # Stay still
      return(car)
    }
  }
  
  path <- astar(x, y, goal_x, goal_y, roads$hroads, roads$vroads)
  
  if (length(path) > 0) {
    next_pos <- path[[1]]
    if (next_pos$x > x) car$nextMove <- 6      # East
    else if (next_pos$x < x) car$nextMove <- 4 # West
    else if (next_pos$y > y) car$nextMove <- 8 # North
    else if (next_pos$y < y) car$nextMove <- 2 # South
    else car$nextMove <- 5                     # Stay still
  } else {
    car$nextMove <- 5  # Stay still if no path found
  }
  
  return(car)
}



runDeliveryMan(myFunction)