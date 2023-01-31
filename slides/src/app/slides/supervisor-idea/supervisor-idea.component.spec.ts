import { ComponentFixture, TestBed } from '@angular/core/testing';

import { SupervisorIdeaComponent } from './supervisor-idea.component';

describe('SupervisorIdeaComponent', () => {
  let component: SupervisorIdeaComponent;
  let fixture: ComponentFixture<SupervisorIdeaComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ SupervisorIdeaComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(SupervisorIdeaComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
