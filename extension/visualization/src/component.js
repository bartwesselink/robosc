const dagreD3 = require('dagre-d3');
const d3 = require('d3');

const visualizeVariables = (variables) => {
    if (!variables) return '';

    const names = Object.keys(variables);

    if (names.length > 0) {
        return `
            <table>
            <thead>
                <tr>
                    <th>Variable</th>
                    <th>Value</th>
                </tr>
            </thead>
            <tbody>
                ${names.map(it => `<tr><td>${it}</td><td><pre><code>${variables[it]}</pre></code></td></tr>`).join('\n')}
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
        label += ` if ${transition.guard}`
    }

    if (!!transition.assignment) {
        label += ` do ${transition.assignment}`
    }

    return label.trim();
};

const visualizeBehaviour = (behaviour, currentState) => {
    const base = document.createElement('div');
    if (!behaviour) return base;

    const id = `${Math.random()}`.replace('.', '');
    const baseSize = 400;
    
    base.innerHTML = `<svg width="${baseSize}" height="${baseSize}" id="${id}"><g></g></svg>`;

    const createGraph = (id) => {
        const element = document.getElementById(id);
        const graph = new dagreD3.graphlib.Graph().setGraph({});

        for (const state of behaviour) {
            graph.setNode(state.name, { label: state.name });

            for (const transition of state.transitions) {
                console.log('label', transitionLabel(transition));
                graph.setEdge(state.name, transition.next ?? state.name, { label: transitionLabel(transition) });
            }
        }

        // Round borders
        graph.nodes().forEach(it => {
            const node = graph.node(it);

            node.rx = 10;
            node.ry = 10;
        })

        // Set up the edges
        // g.setEdge("CLOSED", "LISTEN", { label: "open" });

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
    }

    setTimeout(() => createGraph(id));

    base.classList.add('diagram');

    return base;
}

export const visualizeComponent = (component, currentState) => {
    const wrapper = document.createElement('div');

    const title = document.createElement('h2');
    title.innerHTML = component.name;

    const behaviour = visualizeBehaviour(component.behaviour, currentState);

    const variables = document.createElement('div');
    variables.innerHTML = visualizeVariables(currentState?.variables);

    wrapper.appendChild(title);
    wrapper.appendChild(behaviour);
    wrapper.appendChild(variables);

    return wrapper;
};