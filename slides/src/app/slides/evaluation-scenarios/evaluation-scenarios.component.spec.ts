import { ComponentFixture, TestBed } from '@angular/core/testing';

import { EvaluationScenariosComponent } from './evaluation-scenarios.component';

describe('EvaluationScenariosComponent', () => {
  let component: EvaluationScenariosComponent;
  let fixture: ComponentFixture<EvaluationScenariosComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ EvaluationScenariosComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(EvaluationScenariosComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
