import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppRoutingModule } from './app-routing.module';
import { AppComponent } from './app.component';
import { SectionOverviewComponent } from './components/section-overview/section-overview.component';
import { SectionOverviewItemComponent } from './components/section-overview-item/section-overview-item.component';
import { SlideFooterComponent } from './components/slide-footer/slide-footer.component';
import { InlineSVGModule } from 'ng-inline-svg-2';
import { HttpClientModule } from '@angular/common/http';
import { SlideComponent } from './components/slide/slide.component';
import { SectionComponent } from './components/section/section.component';
import { TitleComponent } from './components/title/title.component';
import { ListComponent } from './components/list/list.component';
import { ListItemComponent } from './components/list-item/list-item.component';
import { TwoColumnLayoutComponent } from './components/two-column-layout/two-column-layout.component';
import { GenericFooterComponent } from './components/generic-footer/generic-footer.component';
import { IntroductionProblemComponent } from './slides/introduction-problem/introduction-problem.component';
import { IntroductionDemonstrationRobotComponent } from './slides/introduction-demonstration-robot/introduction-demonstration-robot.component';
import { OpeningComponent } from './slides/opening/opening.component';
import { PreliminariesSupervisoryControlTheoryComponent } from './slides/preliminaries-supervisory-control-theory/preliminaries-supervisory-control-theory.component';
import { EndingComponent } from './slides/ending/ending.component';
import { CitationComponent } from './components/citation/citation.component';
import { CitationsComponent } from './components/citations/citations.component';
import { FigureComponent } from './components/figure/figure.component';
import { PreliminariesRosComponent } from './slides/preliminaries-ros/preliminaries-ros.component';
import { PreliminariesRosMessagesComponent } from './slides/preliminaries-ros-messages/preliminaries-ros-messages.component';
import { PreliminariesRosServicesComponent } from './slides/preliminaries-ros-services/preliminaries-ros-services.component';
import { PreliminariesRosActionsComponent } from './slides/preliminaries-ros-actions/preliminaries-ros-actions.component';
import { PreliminariesCifComponent } from './slides/preliminaries-cif/preliminaries-cif.component';
import { PreliminariesMdeComponent } from './slides/preliminaries-mde/preliminaries-mde.component';
import { OverviewMainIdeaComponent } from './slides/overview-main-idea/overview-main-idea.component';
import { ToolsGenerationComponent } from './slides/tools-generation/tools-generation.component';
import { OverviewScopeComponent } from './slides/overview-scope/overview-scope.component';
import { ToolsEditorsComponent } from './slides/tools-editors/tools-editors.component';
import { ConceptsBaseComponent } from './slides/concepts-base/concepts-base.component';
import { ConceptsComponentsComponent } from './slides/concepts-components/concepts-components.component';
import { ConceptsCommunicationComponent } from './slides/concepts-communication/concepts-communication.component';
import { ConceptsDataComponent } from './slides/concepts-data/concepts-data.component';
import { ConceptsRequirementsComponent } from './slides/concepts-requirements/concepts-requirements.component';
import { SupervisorIdeaComponent } from './slides/supervisor-idea/supervisor-idea.component';
import { SupervisorLimitationsComponent } from './slides/supervisor-limitations/supervisor-limitations.component';
import { ToolsDebuggingComponent } from './slides/tools-debugging/tools-debugging.component';
import { EvaluationScenariosComponent } from './slides/evaluation-scenarios/evaluation-scenarios.component';
import { ConclusionContributionsComponent } from './slides/conclusion-contributions/conclusion-contributions.component';
import { ConclusionVideoComponent } from './slides/conclusion-video/conclusion-video.component';
import { CodeComponent } from './components/code/code.component';
import { OverviewIdeaComponent } from './slides/overview-idea/overview-idea.component';
import { CitationReferenceComponent } from './components/citation-reference/citation-reference.component';
import { OverviewControlLoopComponent } from './slides/overview-control-loop/overview-control-loop.component';
import { RunningCodeExampleComponent } from './components/running-code-example/running-code-example.component';
import { EvaluationLinesOfCodeComponent } from './slides/evaluation-lines-of-code/evaluation-lines-of-code.component';
import { NgChartsModule } from 'ng2-charts';

@NgModule({
  declarations: [
    AppComponent,
    SectionOverviewComponent,
    SectionOverviewItemComponent,
    SlideFooterComponent,
    SlideComponent,
    SectionComponent,
    TitleComponent,
    ListComponent,
    ListItemComponent,
    TwoColumnLayoutComponent,
    GenericFooterComponent,
    IntroductionProblemComponent,
    IntroductionDemonstrationRobotComponent,
    OpeningComponent,
    PreliminariesSupervisoryControlTheoryComponent,
    EndingComponent,
    CitationComponent,
    CitationsComponent,
    FigureComponent,
    PreliminariesRosComponent,
    PreliminariesRosMessagesComponent,
    PreliminariesRosServicesComponent,
    PreliminariesRosActionsComponent,
    PreliminariesCifComponent,
    PreliminariesMdeComponent,
    OverviewMainIdeaComponent,
    ToolsGenerationComponent,
    OverviewScopeComponent,
    ToolsEditorsComponent,
    ConceptsBaseComponent,
    ConceptsComponentsComponent,
    ConceptsCommunicationComponent,
    ConceptsDataComponent,
    ConceptsRequirementsComponent,
    SupervisorIdeaComponent,
    SupervisorLimitationsComponent,
    ToolsDebuggingComponent,
    EvaluationScenariosComponent,
    ConclusionContributionsComponent,
    ConclusionVideoComponent,
    CodeComponent,
    OverviewIdeaComponent,
    CitationReferenceComponent,
    OverviewControlLoopComponent,
    RunningCodeExampleComponent,
    EvaluationLinesOfCodeComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    NgChartsModule,
    HttpClientModule,
    InlineSVGModule.forRoot(),
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
