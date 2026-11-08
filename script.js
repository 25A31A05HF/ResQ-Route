/*
  Optimized Route Planner for Emergency Vehicles
  Simulation: Dijkstra, A*, AVL Tree, Traffic and Ambulance Animation
*/
'use strict';

(() => {
  const $ = (id) => document.getElementById(id);

  const setText = (id, value) => {
    const element = $(id);
    if (element) element.textContent = String(value);
  };

  const nodes = [
    { id: 'A', name: 'Central', x: 8, y: 50 },
    { id: 'B', name: 'North West', x: 25, y: 18 },
    { id: 'C', name: 'North', x: 48, y: 12 },
    { id: 'D', name: 'North East', x: 76, y: 22 },
    { id: 'E', name: 'East', x: 91, y: 50 },
    { id: 'F', name: 'South East', x: 75, y: 80 },
    { id: 'G', name: 'South', x: 48, y: 88 },
    { id: 'H', name: 'South West', x: 23, y: 78 },
    { id: 'I', name: 'West', x: 4, y: 50 },
    { id: 'J', name: 'City Hospital', x: 58, y: 48 },
    { id: 'K', name: 'Medical Centre', x: 35, y: 48 },
    { id: 'L', name: 'Patient Location', x: 48, y: 65 }
  ];

  const roads = [
    ['A','B',4], ['B','C',3], ['C','D',4], ['D','E',3],
    ['E','F',4], ['F','G',3], ['G','H',4], ['H','I',3],
    ['I','A',2], ['A','K',3], ['K','J',3], ['J','L',2],
    ['K','L',3], ['B','K',4], ['C','K',5], ['C','J',4],
    ['D','J',3], ['E','J',4], ['F','J',3], ['G','L',3],
    ['H','L',4], ['A','L',6], ['B','I',5], ['D','F',5]
  ];

  const hospitals = [
    { id: 1, name: 'City General Hospital', node: 'J', specialty: 'General', beds: 30 },
    { id: 2, name: 'Sunrise Heart Hospital', node: 'C', specialty: 'Cardiology', beds: 12 },
    { id: 3, name: 'Care & Cure Hospital', node: 'K', specialty: 'General', beds: 20 },
    { id: 4, name: 'LifeLine Trauma Centre', node: 'F', specialty: 'Trauma', beds: 10 },
    { id: 5, name: 'Mother & Child Hospital', node: 'H', specialty: 'Maternity', beds: 15 },
    { id: 6, name: 'Neuro Care Institute', node: 'D', specialty: 'Neurology', beds: 8 }
  ];

  const blockedRoads = new Set();
  const traffic = new Map();

  let currentRoute = [];
  let animationTimer = null;
  let rotationCount = 0;
  let avlRoot = null;

  function edgeKey(a, b) {
    return [a, b].sort().join('-');
  }

  function buildGraph() {
    const graph = new Map(nodes.map(node => [node.id, []]));

    roads.forEach(([a, b, distance]) => {
      const key = edgeKey(a, b);
      if (blockedRoads.has(key)) return;

      const cost = distance * (traffic.get(key) || 1);

      graph.get(a).push({ to: b, cost, distance, key });
      graph.get(b).push({ to: a, cost, distance, key });
    });

    return graph;
  }

  function heuristic(a, b) {
    const first = nodes.find(node => node.id === a);
    const second = nodes.find(node => node.id === b);

    if (!first || !second) return 0;

    return Math.hypot(
      first.x - second.x,
      first.y - second.y
    ) / 12;
  }

  class PriorityQueue {
    constructor() {
      this.items = [];
    }

    get size() {
      return this.items.length;
    }

    push(item) {
      this.items.push(item);

      let index = this.items.length - 1;

      while (index > 0) {
        const parent = Math.floor((index - 1) / 2);

        if (this.items[parent].priority <= this.items[index].priority) {
          break;
        }

        [this.items[parent], this.items[index]] =
          [this.items[index], this.items[parent]];

        index = parent;
      }
    }

    pop() {
      if (!this.items.length) return null;

      const minimum = this.items[0];
      const last = this.items.pop();

      if (this.items.length) {
        this.items[0] = last;

        let index = 0;

        while (true) {
          const left = index * 2 + 1;
          const right = index * 2 + 2;
          let smallest = index;

          if (
            left < this.items.length &&
            this.items[left].priority < this.items[smallest].priority
          ) {
            smallest = left;
          }

          if (
            right < this.items.length &&
            this.items[right].priority < this.items[smallest].priority
          ) {
            smallest = right;
          }

          if (smallest === index) break;

          [this.items[index], this.items[smallest]] =
            [this.items[smallest], this.items[index]];

          index = smallest;
        }
      }

      return minimum;
    }
  }

  function findPath(start, goal, algorithm) {
    const graph = buildGraph();

    if (!graph.has(start) || !graph.has(goal)) return null;

    const distances = new Map(nodes.map(node => [node.id, Infinity]));
    const previous = new Map();
    const visited = new Set();
    const queue = new PriorityQueue();

    distances.set(start, 0);

    queue.push({
      id: start,
      priority: algorithm === 'A*' ? heuristic(start, goal) : 0
    });

    let explored = 0;

    while (queue.size > 0) {
      const current = queue.pop();

      if (!current || visited.has(current.id)) continue;

      visited.add(current.id);
      explored++;

      if (current.id === goal) break;

      (graph.get(current.id) || []).forEach(edge => {
        if (visited.has(edge.to)) return;

        const candidate = distances.get(current.id) + edge.cost;

        if (candidate < distances.get(edge.to)) {
          distances.set(edge.to, candidate);
          previous.set(edge.to, current.id);

          const estimate =
            algorithm === 'A*' ? heuristic(edge.to, goal) : 0;

          queue.push({
            id: edge.to,
            priority: candidate + estimate
          });
        }
      });
    }

    if (!Number.isFinite(distances.get(goal))) return null;

    const path = [];
    let cursor = goal;

    while (cursor) {
      path.unshift(cursor);

      if (cursor === start) break;

      cursor = previous.get(cursor);
    }

    if (path[0] !== start) return null;

    return {
      path,
      cost: distances.get(goal),
      explored
    };
  }

  // AVL TREE

  class AVLNode {
    constructor(hospital) {
      this.hospital = hospital;
      this.left = null;
      this.right = null;
      this.height = 1;
    }
  }

  function height(node) {
    return node ? node.height : 0;
  }

  function updateHeight(node) {
    node.height = 1 + Math.max(height(node.left), height(node.right));
  }

  function balanceFactor(node) {
    return node ? height(node.left) - height(node.right) : 0;
  }

  function rotateRight(y) {
    const x = y.left;
    const middle = x.right;

    x.right = y;
    y.left = middle;

    updateHeight(y);
    updateHeight(x);

    rotationCount++;

    return x;
  }

  function rotateLeft(x) {
    const y = x.right;
    const middle = y.left;

    y.left = x;
    x.right = middle;

    updateHeight(x);
    updateHeight(y);

    rotationCount++;

    return y;
  }

  function insertAVL(root, hospital) {
    if (!root) return new AVLNode(hospital);

    const comparison = hospital.name.localeCompare(root.hospital.name);

    if (comparison < 0) {
      root.left = insertAVL(root.left, hospital);
    } else if (comparison > 0) {
      root.right = insertAVL(root.right, hospital);
    } else {
      return root;
    }

    updateHeight(root);

    const balance = balanceFactor(root);

    if (
      balance > 1 &&
      hospital.name.localeCompare(root.left.hospital.name) < 0
    ) {
      return rotateRight(root);
    }

    if (
      balance < -1 &&
      hospital.name.localeCompare(root.right.hospital.name) > 0
    ) {
      return rotateLeft(root);
    }

    if (
      balance > 1 &&
      hospital.name.localeCompare(root.left.hospital.name) > 0
    ) {
      root.left = rotateLeft(root.left);
      return rotateRight(root);
    }

    if (
      balance < -1 &&
      hospital.name.localeCompare(root.right.hospital.name) < 0
    ) {
      root.right = rotateRight(root.right);
      return rotateLeft(root);
    }

    return root;
  }

  hospitals.forEach(hospital => {
    avlRoot = insertAVL(avlRoot, hospital);
  });

  function getAVLRecords(root, result) {
    if (!root) return;

    getAVLRecords(root.left, result);
    result.push(root.hospital);
    getAVLRecords(root.right, result);
  }

  function renderHospitalList(list) {
    const container = $('hospitalList');
    if (!container) return;

    container.replaceChildren();

    if (!list.length) {
      const empty = document.createElement('p');
      empty.textContent = 'No hospitals found for this specialty.';
      container.appendChild(empty);
      return;
    }

    list.forEach(hospital => {
      const item = document.createElement('div');
      item.className = 'hospital-item';

      const title = document.createElement('strong');
      title.textContent = hospital.name;

      const details = document.createElement('span');
      details.textContent =
        ` ${hospital.specialty} • Node ${hospital.node} • ${hospital.beds} beds`;

      item.append(title, details);
      container.appendChild(item);
    });
  }

  function getSelectedValue(id, fallback) {
    const element = $(id);
    return element && element.value ? element.value : fallback;
  }

  function setStatus(message) {
    ['mapStatus', 'routeMessage', 'ambulanceStatus'].forEach(id => {
      const element = $(id);
      if (element) element.textContent = message;
    });
  }

  function updateStats() {
    setText('hospitalCount', hospitals.length);
    setText('roadCount', roads.length - blockedRoads.size);
    setText('treeRecords', hospitals.length);
    setText('treeHeight', height(avlRoot));
    setText('treeRotations', rotationCount);

    renderHospitalList(hospitals);
  }

  function createMap() {
    const grid = $('cityGrid');
    if (!grid) return;

    grid.replaceChildren();
    grid.style.position = 'relative';
    grid.style.minHeight = grid.style.minHeight || '360px';
    grid.style.overflow = 'hidden';

    roads.forEach(([a, b]) => {
      const first = nodes.find(node => node.id === a);
      const second = nodes.find(node => node.id === b);

      const line = document.createElement('div');
      line.className = 'road-line';
      line.dataset.road = edgeKey(a, b);

      const dx = second.x - first.x;
      const dy = second.y - first.y;

      line.style.position = 'absolute';
      line.style.left = `${first.x}%`;
      line.style.top = `${first.y}%`;
      line.style.width = `${Math.hypot(dx, dy)}%`;
      line.style.height = '3px';
      line.style.background =
        blockedRoads.has(edgeKey(a, b)) ? '#dc2626' : '#94a3b8';
      line.style.transformOrigin = '0 50%';
      line.style.transform =
        `rotate(${Math.atan2(dy, dx) * 180 / Math.PI}deg)`;
      line.style.zIndex = '1';

      grid.appendChild(line);
    });

    nodes.forEach(node => {
      const marker = document.createElement('div');

      marker.className = 'map-node';
      marker.dataset.node = node.id;
      marker.textContent = node.id;
      marker.title = node.name;

      marker.style.position = 'absolute';
      marker.style.left = `${node.x}%`;
      marker.style.top = `${node.y}%`;
      marker.style.transform = 'translate(-50%, -50%)';
      marker.style.zIndex = '3';
      marker.style.width = '28px';
      marker.style.height = '28px';
      marker.style.borderRadius = '50%';
      marker.style.display = 'grid';
      marker.style.placeItems = 'center';
      marker.style.background =
        hospitals.some(h => h.node === node.id) ? '#16a34a' : '#2563eb';
      marker.style.color = '#fff';
      marker.style.fontWeight = '700';
      marker.style.fontSize = '12px';

      grid.appendChild(marker);
    });

    const ambulance = document.createElement('div');

    ambulance.id = 'ambulanceMarker';
    ambulance.textContent = '🚑';
    ambulance.style.position = 'absolute';
    ambulance.style.left = '8%';
    ambulance.style.top = '50%';
    ambulance.style.transform = 'translate(-50%, -50%)';
    ambulance.style.zIndex = '5';
    ambulance.style.fontSize = '24px';

    grid.appendChild(ambulance);
  }

  function drawRoute(path) {
    document.querySelectorAll('.map-node').forEach(marker => {
      marker.style.outline =
        path.includes(marker.dataset.node) ? '3px solid #f59e0b' : 'none';
    });

    const grid = $('cityGrid');
    if (!grid) return;

    grid.querySelectorAll('.route-label').forEach(element => {
      element.remove();
    });

    path.forEach((nodeId, index) => {
      const node = nodes.find(entry => entry.id === nodeId);
      if (!node) return;

      const label = document.createElement('div');

      label.className = 'route-label';
      label.textContent = String(index + 1);
      label.style.position = 'absolute';
      label.style.left = `${node.x}%`;
      label.style.top = `${node.y + 5}%`;
      label.style.transform = 'translateX(-50%)';
      label.style.zIndex = '4';
      label.style.fontSize = '11px';
      label.style.fontWeight = 'bold';
      label.style.color = '#b45309';

      grid.appendChild(label);
    });
  }

  function pathDistance(path) {
    let total = 0;

    for (let i = 0; i < path.length - 1; i++) {
      const road = roads.find(([a, b]) =>
        edgeKey(a, b) === edgeKey(path[i], path[i + 1])
      );

      if (road) total += road[2];
    }

    return total;
  }

  function stopAnimation() {
    if (animationTimer !== null) {
      window.clearInterval(animationTimer);
      animationTimer = null;
    }
  }

  function calculateRoute() {
    stopAnimation();

    const start = getSelectedValue('startSelect', 'A');
    const patient = getSelectedValue('patientSelect', 'L');
    const algorithm = getSelectedValue('algorithmSelect', 'Dijkstra');

    const chosenHospital = hospitals
      .map(hospital => ({
        hospital,
        result: findPath(patient, hospital.node, algorithm)
      }))
      .filter(entry => entry.result)
      .sort((a, b) => a.result.cost - b.result.cost)[0];

    const toPatient = findPath(start, patient, algorithm);

    if (!toPatient || !chosenHospital) {
      currentRoute = [];

      setStatus('No available route. Try another location or clear blocked roads.');
      setText('recommendedHospital', 'No route available');
      setText('hospitalDetails', 'Please check the selected locations.');
      setText('totalCost', '--');
      setText('distance', '--');
      setText('nodesExplored', toPatient ? toPatient.explored : 0);

      return;
    }

    const fullPath = toPatient.path.concat(
      chosenHospital.result.path.slice(1)
    );

    const totalCost = toPatient.cost + chosenHospital.result.cost;
    const totalDistance = pathDistance(fullPath);

    currentRoute = fullPath;

    drawRoute(fullPath);

    setText('recommendedHospital', chosenHospital.hospital.name);

    setText(
      'hospitalDetails',
      `${chosenHospital.hospital.specialty} • Node ${chosenHospital.hospital.node} • ${chosenHospital.hospital.beds} beds`
    );

    setText('totalCost', `${totalCost.toFixed(2)} km-equivalent`);
    setText('distance', `${totalDistance.toFixed(1)} km`);

    setText(
      'nodesExplored',
      toPatient.explored + chosenHospital.result.explored
    );

    const dijkstra = findPath(start, patient, 'Dijkstra');
    const astar = findPath(start, patient, 'A*');

    setText('dijkstraNodes', dijkstra ? dijkstra.explored : '--');
    setText('dijkstraCost', dijkstra ? dijkstra.cost.toFixed(2) : '--');
    setText('astarNodes', astar ? astar.explored : '--');
    setText('astarCost', astar ? astar.cost.toFixed(2) : '--');

    setText(
      'comparisonSummary',
      `Best route calculated using ${algorithm}. Route: ${fullPath.join(' → ')}`
    );

    setStatus(`Route ready: ${start} → ${patient} → ${chosenHospital.hospital.node}`);
  }

  function animateAmbulance() {
    if (!currentRoute.length) calculateRoute();
    if (!currentRoute.length) return;

    stopAnimation();

    const marker = $('ambulanceMarker');
    const grid = $('cityGrid');

    if (!marker || !grid) {
      setStatus('Route calculated. Add a cityGrid element to display animation.');
      return;
    }

    let index = 0;

    function move() {
      const node = nodes.find(entry => entry.id === currentRoute[index]);

      if (node) {
        marker.style.left = `${node.x}%`;
        marker.style.top = `${node.y}%`;
      }

      if (index >= currentRoute.length - 1) {
        stopAnimation();
        setStatus('Ambulance reached the selected hospital.');
        return;
      }

      index++;
    }

    move();
    animationTimer = window.setInterval(move, 700);

    setStatus('Ambulance is moving along the calculated route...');
  }

  function toggleTraffic() {
    const available = roads.filter(([a, b]) =>
      !blockedRoads.has(edgeKey(a, b))
    );

    if (!available.length) return;

    const selected = available[Math.floor(Math.random() * available.length)];
    const key = edgeKey(selected[0], selected[1]);

    if (traffic.has(key)) {
      traffic.delete(key);
    } else {
      traffic.set(key, 2.5);
    }

    createMap();

    setText(
      'trafficStatus',
      traffic.size
        ? `${traffic.size} road(s) have simulated heavy traffic.`
        : 'Traffic is normal.'
    );

    calculateRoute();
  }

  function toggleBlockMode() {
    const available = roads.filter(([a, b]) =>
      !blockedRoads.has(edgeKey(a, b))
    );

    if (available.length) {
      const road = available[Math.floor(Math.random() * available.length)];
      blockedRoads.add(edgeKey(road[0], road[1]));
    } else if (blockedRoads.size) {
      blockedRoads.clear();
    }

    createMap();

    setText(
      'trafficStatus',
      `${blockedRoads.size} road(s) blocked in this simulation.`
    );

    calculateRoute();
  }

  function clearConditions() {
    blockedRoads.clear();
    traffic.clear();

    stopAnimation();
    createMap();

    setText('trafficStatus', 'Traffic is normal. No roads are blocked.');

    calculateRoute();
  }

  function searchHospital() {
    const specialty = getSelectedValue('avlSpecialty', 'All');
    const records = [];

    getAVLRecords(avlRoot, records);

    const matches = specialty === 'All'
      ? records
      : records.filter(hospital => hospital.specialty === specialty);

    renderHospitalList(matches);

    setText(
      'avlStatus',
      `${matches.length} hospital(s) found using the AVL hospital index.`
    );
  }

  function populateSelect(id, includeAll) {
    const select = $(id);

    if (!select || select.options.length) return;

    if (includeAll) {
      const all = document.createElement('option');

      all.value = 'All';
      all.textContent = 'All specialties';

      select.appendChild(all);
    }

    if (id === 'avlSpecialty') {
      [...new Set(hospitals.map(h => h.specialty))]
        .sort()
        .forEach(specialty => {
          const option = document.createElement('option');

          option.value = specialty;
          option.textContent = specialty;

          select.appendChild(option);
        });

      return;
    }

    nodes.forEach(node => {
      const option = document.createElement('option');

      option.value = node.id;
      option.textContent = `${node.id} — ${node.name}`;

      select.appendChild(option);
    });
  }

  function bind(id, eventName, handler) {
    const element = $(id);

    if (element) {
      element.addEventListener(eventName, handler);
    }
  }

  function initialize() {
    populateSelect('startSelect', false);
    populateSelect('patientSelect', false);
    populateSelect('avlSpecialty', true);

    updateStats();
    createMap();

    bind('findRouteBtn', 'click', calculateRoute);
    bind('animateBtn', 'click', animateAmbulance);
    bind('trafficBtn', 'click', toggleTraffic);
    bind('blockModeBtn', 'click', toggleBlockMode);
    bind('clearBtn', 'click', clearConditions);
    bind('searchHospitalBtn', 'click', searchHospital);

    bind('algorithmSelect', 'change', calculateRoute);
    bind('startSelect', 'change', calculateRoute);
    bind('patientSelect', 'change', calculateRoute);

    calculateRoute();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initialize, { once: true });
  } else {
    initialize();
  }
})();