import { ComponentFixture, TestBed } from '@angular/core/testing';

import { SupervisorLimitationsComponent } from './supervisor-limitations.component';

describe('SupervisorLimitationsComponent', () => {
  let component: SupervisorLimitationsComponent;
  let fixture: ComponentFixture<SupervisorLimitationsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ SupervisorLimitationsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(SupervisorLimitationsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
