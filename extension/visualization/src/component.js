const dagreD3 = require('dagre-d3');
const d3 = require('d3');

const componentId = (component) => `component-${component.name}`;
const variableId = (variable, component) => `${componentId(component)}-variable-${variable}`;

const visualizeVariables = (component) => {
    const names = component.behaviour?.variables;

    if (names?.length > 0) {
        return `
            <table>
            <thead>
                <tr>
                    <th width="50%">Variable</th>
                    <th width="50%">Value</th>
                </tr>
            </thead>
            <tbody>
                ${names.map(it => `<tr><td>${it}</td><td><pre><code id="${variableId(it, component)}"></pre></code></td></tr>`).join('\n')}
            </tbody>
            </table>
        `
    }

    return '';
}

const transitionLabel = (transition) => {
    let label = '';

    if (transition.type !== 'tau') {
        label += `${transition.communication}`
    }

    if (!!transition.guard) {
        label += ` [${transition.guard}]`
    }

    if (!!transition.assignment) {
        label += ` / ${transition.assignment}`
    }

    return label.trim();
};

const visualizeBehaviour = (component) => {
    const base = document.createElement('div');
    const behaviour = component.behaviour;

    if (!behaviour) return base;

    const id = componentId(component);
    const baseSize = 400;

    base.innerHTML = `<svg width="${baseSize}" height="0" id="${id}"><g></g></svg>`;

    const createGraph = (id) => {
        const element = document.getElementById(id);
        const graph = new dagreD3.graphlib.Graph().setGraph({});

        for (const state of behaviour.states) {
            graph.setNode(state.name, { label: state.name });

            for (const transition of state.transitions) {
                transition.className = `${transition.id}_${String(Math.random()).replace('.', '')}`;

                graph.setEdge(state.name, transition.next ?? state.name, { label: transitionLabel(transition), curve: d3.curveBasis, class: transition.className });
            }
        }

        // Round borders
        graph.nodes().forEach(it => {
            const node = graph.node(it);

            node.rx = 10;
            node.ry = 10;
        })

        const svg = d3.select(element),
            inner = svg.select('g');

        const zoom = d3.zoom().on('zoom', function () {
            inner.attr('transform', d3.event.transform);
        });
        svg.call(zoom);

        const render = new dagreD3.render();

        render(inner, graph);

        const initialScale = 0.75;

        svg.attr('width', base.parentElement.getBoundingClientRect().width);
        svg.call(zoom.transform, d3.zoomIdentity.translate((svg.attr('width') - graph.graph().width * initialScale) / 2, 20).scale(initialScale));

        svg.attr('height', graph.graph().height * initialScale + 80);

        component.graph = graph;
        component.svg = element;
    }

    setTimeout(() => createGraph(id));

    base.classList.add('diagram');

    return base;
}

export const visualizeComponent = (component) => {
    const wrapper = document.createElement('div');

    const title = document.createElement('h2');
    title.innerHTML = component.name;

    const behaviour = visualizeBehaviour(component);

    const variables = document.createElement('div');
    variables.innerHTML = visualizeVariables(component);

    wrapper.appendChild(title);
    wrapper.appendChild(behaviour);
    wrapper.appendChild(variables);

    if (!component.behaviour && !component.variables) {
        const empty = document.createElement('div');
        empty.innerHTML = 'There is no behaviour or variable information available.';

        wrapper.appendChild(empty);
    }

    return wrapper;
};

const equalTransition = (one, two) => {
    if (!one || !two) return false;

    return one.toLowerCase().replace(/\./g, '_').replace(/_$/, '') === two.toLowerCase().replace(/\./g, '_').trim('_', '').replace(/_$/, '');
}

export const updateState = (definition, current, transitions) => {
    const behaviourComponents = Object.keys(current);

    for (const name of behaviourComponents) {
        const component = definition.components.find(it => it.name === name);

        component.behaviour
            ?.variables
            .forEach(variable => document.getElementById(variableId(variable, component)).innerHTML = current[name].variables[variable]);


        // Clear current state
        component.svg?.querySelectorAll('.active').forEach(it => it.classList.remove('active'));

        const currentState = component.graph.node(current[name].state).elem;
        currentState.classList.add('active');

        // Check all transitions that should be updated
        // Pick last known state, and match it with all taken transitions
        if (component.lastKnownState) {
            const stateObject = component.behaviour?.states?.find(it => it.name === component.lastKnownState);
            const takenTransitions = stateObject.transitions.filter(it => transitions.some(taken => equalTransition(taken, it.id)));

            takenTransitions.forEach(it => {
                if (it.timeout) clearTimeout(it.timeout);

                const element = component.svg?.querySelector(`.${it.className}`);

                it.timeout = setTimeout(() => {
                    element.classList.remove('taken');
                }, 500);

                element.classList.add('taken');
            });
        }

        component.lastKnownState = current[name].state;
    }
}